/**
 * E1.5 过调制（阶段 1，开环）
 *
 * 目的：验证"过调制能多榨约 10% 电压"以及它对波形的影响。
 *
 * 理论：
 *   线性区（m ≤ 1）    相电压基波峰值 = m · Vdc/√3
 *   六步极限（m→∞）    相电压基波峰值 = 2·Vdc/π = 0.6366·Vdc
 *   两者之比                        = (2/π)/(1/√3) = 1.1027 → 多 10.27%
 *   调制度 m 对应的理论基波（mode-I 近似）：
 *     m ≤ 1        :  A1 = m·Vdc/√3
 *     1 < m ≤ 1.1547: A1 ≈ Vdc/√3 · (1 + (m−1)·( (2/π)/(1/√3) − 1 ) / (2/√3 − 1) )
 *
 * 本实验用一个【单周期 DFT】(Goertzel) 实时测出实际输出的相电压基波幅值，
 * 与理论曲线对照 —— 这样"多榨 10%"不再是书上的一句话，而是你自己测出来的数。
 *
 * 串口命令：
 *   m <0.8~1.3>      调制度
 *   sweep 0|1        自动从 0.8 扫到 1.30
 *   f <Hz>           电频率（DFT 需要它精确）
 *   start / stop
 *   dft              打印当前 DFT 读数与理论值对比
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static PwmDriver    pwm;
static CurrentSense cs;
static Console      con;

static bool  running = false;
static bool  sweeping = false;
static float fe   = 1.0f;
static float m    = 0.9f;
static float vdc  = VDC_DEFAULT;
static float thetaE = 0.0f;
static uint32_t tPrev = 0;

/// 单周期 DFT：累加 (duty_a-0.5)*Vdc 的基波分量
struct Dft1 {
  double re = 0, im = 0;
  int    n  = 0;
  void reset() { re = im = 0; n = 0; }
  void acc(float x, float th) {
    re += (double)x * cosf(th);
    im += (double)x * sinf(th);
    n++;
  }
  float amp() const { return n > 2 ? (float)(2.0 * sqrt(re * re + im * im) / n) : 0.0f; }
};
static Dft1 dft;

/// 理论基波幅值（V）
static float theoryA1(float mm) {
  const float lin = vdc * ONE_BY_SQRT3_F;
  if (mm <= 1.0f) return mm * lin;
  const float aSixStep = 2.0f * vdc / PI_F;          // 0.6366 Vdc
  const float mSixStep = 2.0f * ONE_BY_SQRT3_F;      // 1.1547
  if (mm >= mSixStep) return aSixStep;
  const float k = (mm - 1.0f) / (mSixStep - 1.0f);
  return lin + k * (aSixStep - lin);
}

static void cmd_m(int argc, char** argv) {
  m = Console::argf(argv, argc, 1, m);
  Console::printfln("m = %.3f  理论基波 = %.3f V (线性区上限 %.3f V)", m, theoryA1(m),
                    vdc * ONE_BY_SQRT3_F);
}

static void cmd_f(int argc, char** argv) { fe = Console::argf(argv, argc, 1, fe); }
static void cmd_vdc(int argc, char** argv) { vdc = Console::argf(argv, argc, 1, vdc); }
static void cmd_sweep(int argc, char** argv) {
  sweeping = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("sweep %s", sweeping ? "ON (0.80 → 1.30)" : "OFF");
}
static void cmd_start(int, char**) { thetaE = 0; tPrev = micros(); pwm.enable(true); running = true; dft.reset(); }
static void cmd_stop(int, char**)  { running = false; pwm.coast(); }

static void cmd_dft(int, char**) {
  Console::printfln("m=%.3f  实测基波=%.3f V  理论=%.3f V  误差=%+.1f%%", m, dft.amp(),
                    theoryA1(m), 100.0f * (dft.amp() / fmaxf(theoryA1(m), 1e-6f) - 1.0f));
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E1.5 过调制 ===\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(20000.0f);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(512);

  con.add("m",     cmd_m,     "m <0.8~1.3>");
  con.add("f",     cmd_f,     "f <Hz>");
  con.add("vdc",   cmd_vdc,   "vdc <V>");
  con.add("sweep", cmd_sweep, "sweep 0|1 自动扫描");
  con.add("start", cmd_start, "开始");
  con.add("stop",  cmd_stop,  "停止");
  con.add("dft",   cmd_dft,   "打印基波实测/理论对比");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "v_a");
  telemetry::setChannelName(2, "m");
  telemetry::setChannelName(3, "A1_meas");
  telemetry::setChannelName(4, "A1_theory");
  telemetry::setChannelName(5, "theta_e");

  Console::print("命令：m <0.8~1.3> / sweep 1 / start / stop / dft\r\n");
}

void loop() {
  con.poll();

  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  // 自动扫描：每 2 秒升一次 m
  if (running && sweeping) {
    static uint32_t tLast = 0;
    if (now - tLast > 2000000u) {
      tLast = now;
      m += 0.05f;
      if (m > 1.3001f) m = 0.80f;
      Console::printfln("[sweep] m=%.2f  theory=%.3f V  meas=%.3f V", m, theoryA1(m), dft.amp());
    }
  }

  Duty d{0.5f, 0.5f, 0.5f};
  float vA = 0.0f;
  if (running) {
    const float prevTheta = thetaE;
    thetaE = wrap2Pi(thetaE + TWO_PI_F * fe * dt);

    const float V = m * vdc * ONE_BY_SQRT3_F;
    const AlphaBeta v{V * cosf(thetaE), V * sinf(thetaE)};
    d = modulate(Mod::SVPWM7, v, vdc);
    clampDuty(d);
    pwm.setDuty(d);

    vA = (d.a - 0.5f) * vdc;

    // 一个电周期为一个 DFT 窗
    if (thetaE < prevTheta) {
      dft.reset();
    }
    dft.acc(vA, thetaE);
  }

  float ia, ib, ic;
  cs.read(ia, ib, ic);
  telemetry::set(0, ia);
  telemetry::set(1, vA);
  telemetry::set(2, m);
  telemetry::set(3, dft.amp());
  telemetry::set(4, theoryA1(m));
  telemetry::set(5, thetaE);
  telemetry::tick();
}
