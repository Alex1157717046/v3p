/**
 * E1.3 从零实现 SVPWM 七段式（阶段 1，开环）★ 核心实验
 *
 * 目的：亲手实现扇区判断 + 伏秒平衡，并验证电压利用率比 SPWM 高 15.47%。
 *
 * 验收：
 *   ① 线电压为 3 电平波形
 *   ② 线性区极限相电压峰值 ≈ Vdc/√3（= 1.1547·Vdc/2）
 *   ③ 相对 SPWM 提升 ≥ 15%
 *
 * 实现见 lib/v3plab/v3p_math.h: svpwm7()（显式扇区法）与 svpwmMinMax()（载波比较法）。
 * 两者在离线校验中逐点完全相等 —— 用 `verify` 命令可在板上再验一次。
 *
 * 串口命令：
 *   f <Hz>              电频率
 *   m <0~1.3>           调制度（相对 Vdc/√3，m=1 即线性区极限）
 *   vdc <V>             母线电压标称
 *   mode spwm|svpwm7|svpwm5|mm
 *   start / stop
 *   verify              运行 svpwm7 vs min-max 的在线一致性检查
 *   sec                 打印扇区判断表（0~360° 每 30°）
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static PwmDriver    pwm;
static CurrentSense cs;
static Console      con;

static bool  running = false;
static float fe   = 1.0f;
static float m    = 0.5f;
static float vdc  = VDC_DEFAULT;
static Mod   mode = Mod::SVPWM7;
static float thetaE = 0.0f;
static uint32_t tPrev = 0;
static int   lastSector = 0;

static void cmd_f(int argc, char** argv) { fe = Console::argf(argv, argc, 1, fe); }

static void cmd_m(int argc, char** argv) {
  m = Console::argf(argv, argc, 1, m);
  Console::printfln("m = %.3f  相电压峰值 = %.3f V  (线性区上限 %.3f V)", m,
                    m * vdc * ONE_BY_SQRT3_F, vdc * ONE_BY_SQRT3_F);
}

static void cmd_vdc(int argc, char** argv) { vdc = Console::argf(argv, argc, 1, vdc); }

static void cmd_mode(int argc, char** argv) {
  if (argc < 2) { Console::printfln("mode = %s", modName(mode)); return; }
  if      (!strcmp(argv[1], "spwm"))   mode = Mod::SPWM;
  else if (!strcmp(argv[1], "svpwm7")) mode = Mod::SVPWM7;
  else if (!strcmp(argv[1], "svpwm5")) mode = Mod::SVPWM5;
  else if (!strcmp(argv[1], "mm"))     mode = Mod::SVPWMMinMax;
  else { Console::print("unknown mode\r\n"); return; }
  Console::printfln("mode = %s", modName(mode));
}

static void cmd_start(int, char**) { thetaE = 0; tPrev = micros(); pwm.enable(true); running = true; }
static void cmd_stop(int, char**)  { running = false; pwm.coast(); }

/// 在线一致性验证：逐点比较显式扇区法与 min-max 法
static void cmd_verify(int, char**) {
  double maxErr = 0.0;
  int    cnt    = 0;
  for (int k = 0; k < 720; k++) {
    const float th = (float)k * 0.5f * PI_F / 180.0f;
    for (float mm = 0.1f; mm <= 1.0f; mm += 0.3f) {
      const float V = mm * vdc * ONE_BY_SQRT3_F;
      const AlphaBeta v{V * cosf(th), V * sinf(th)};
      const Duty d1 = svpwm7(v, vdc);
      const Duty d2 = svpwmMinMax(v, vdc);
      maxErr = fmax(maxErr, fabs(d1.a - d2.a));
      maxErr = fmax(maxErr, fabs(d1.b - d2.b));
      maxErr = fmax(maxErr, fabs(d1.c - d2.c));
      cnt++;
    }
  }
  Console::printfln("verify: %d 点, max|svpwm7 - minmax| = %.9f  (应≈0)", cnt, maxErr);
}

/// 打印扇区判断表
static void cmd_sec(int, char**) {
  Console::print("theta(deg)  sector  T1/T2区域\r\n");
  for (int deg = 0; deg < 360; deg += 30) {
    const float th = (float)deg * PI_F / 180.0f;
    const float V  = 0.5f * vdc * ONE_BY_SQRT3_F;
    int   sec = 0;
    float mm  = 0.0f;
    svpwm7(AlphaBeta{V * cosf(th), V * sinf(th)}, vdc, &sec, &mm);
    Console::printfln("%5d        %d", deg, sec);
  }
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E1.3 SVPWM 七段式 ===\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(20000.0f);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(512);

  con.add("f",      cmd_f,      "f <Hz>");
  con.add("m",      cmd_m,      "m <0~1.3> 调制度(相对 Vdc/sqrt3)");
  con.add("vdc",    cmd_vdc,    "vdc <V>");
  con.add("mode",   cmd_mode,   "mode spwm|svpwm7|svpwm5|mm");
  con.add("start",  cmd_start,  "开始");
  con.add("stop",   cmd_stop,   "停止");
  con.add("verify", cmd_verify, "在线验证两种 SVPWM 实现一致");
  con.add("sec",    cmd_sec,    "打印扇区判断表");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "duty_a");
  telemetry::setChannelName(2, "duty_b");
  telemetry::setChannelName(3, "duty_c");
  telemetry::setChannelName(4, "sector");
  telemetry::setChannelName(5, "theta_e");

  Console::print("命令：f / m / mode / start / stop / verify / sec\r\n");
}

void loop() {
  con.poll();

  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  Duty d{0.5f, 0.5f, 0.5f};
  if (running) {
    thetaE = wrap2Pi(thetaE + TWO_PI_F * fe * dt);
    const float V = m * vdc * ONE_BY_SQRT3_F;
    const AlphaBeta v{V * cosf(thetaE), V * sinf(thetaE)};
    int sec = 0;
    d = modulate(mode, v, vdc, 0, &sec, nullptr);
    lastSector = sec;
    clampDuty(d);
    pwm.setDuty(d);
  }

  float ia, ib, ic;
  cs.read(ia, ib, ic);
  telemetry::set(0, ia);
  telemetry::set(1, d.a);
  telemetry::set(2, d.b);
  telemetry::set(3, d.c);
  telemetry::set(4, (float)lastSector);
  telemetry::set(5, thetaE);
  telemetry::tick();
}
