/**
 * E1.2 从零实现 SPWM（阶段 1，开环）
 *
 * 目的：建立正弦调制基线，为 E1.3 对比 SVPWM 做准备。
 *
 * 原理：duty_x = 0.5 + v_x/Vdc，v_x 为三相正弦（幅值 ≤ Vdc/2 才是线性区）
 *
 * 验收：线性区极限相电压峰值 ≈ Vdc/2；再大就进入过调制（顶到 0/1）
 *
 * 串口命令：
 *   f <Hz>         电频率
 *   amp <V>        相电压峰值（相对 Vdc/2 的比例也可用 m 设置）
 *   m <0~1.15>     调制度（相对 SPWM 线性区上限 Vdc/2；>1 即过调制）
 *   vdc <V>        母线电压标称值
 *   start / stop
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static PwmDriver    pwm;
static CurrentSense cs;
static Console      con;

static bool  running = false;
static float fe      = 1.0f;
static float amp     = 1.0f;      // 相电压峰值 V
static float vdc     = VDC_DEFAULT;
static float thetaE  = 0.0f;
static uint32_t tPrev = 0;

static void cmd_f(int argc, char** argv) {
  fe = Console::argf(argv, argc, 1, fe);
  Console::printfln("fe = %.3f Hz", fe);
}

static void cmd_amp(int argc, char** argv) {
  amp = Console::argf(argv, argc, 1, amp);
  Console::printfln("amp = %.3f V  (SPWM 线性区上限 %.3f V, m=%.3f)", amp, vdc * 0.5f,
                    amp / (vdc * 0.5f));
}

static void cmd_m(int argc, char** argv) {
  const float m = Console::argf(argv, argc, 1, 0.5f);
  amp = m * vdc * 0.5f;      // SPWM 的 m=1 对应 Vdc/2
  Console::printfln("m = %.3f  → amp = %.3f V", m, amp);
}

static void cmd_vdc(int argc, char** argv) {
  vdc = Console::argf(argv, argc, 1, vdc);
  Console::printfln("vdc = %.2f V", vdc);
}

static void cmd_start(int, char**) {
  thetaE = 0.0f;
  tPrev = micros();
  pwm.enable(true);
  running = true;
  Console::print("SPWM RUNNING\r\n");
}

static void cmd_stop(int, char**) {
  running = false;
  pwm.coast();
  Console::print("stopped\r\n");
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E1.2 SPWM（开环）===\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(20000.0f);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(512);

  con.add("f",     cmd_f,     "f <Hz>");
  con.add("amp",   cmd_amp,   "amp <V> 相电压峰值");
  con.add("m",     cmd_m,     "m <0~1.15> 调制度(相对 Vdc/2)");
  con.add("vdc",   cmd_vdc,   "vdc <V>");
  con.add("start", cmd_start, "开始");
  con.add("stop",  cmd_stop,  "停止");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "duty_a");
  telemetry::setChannelName(2, "duty_b");
  telemetry::setChannelName(3, "duty_c");
  telemetry::setChannelName(4, "v_phase_pk");
  telemetry::setChannelName(5, "theta_e");

  Console::print("命令：f <Hz> / amp <V> / m <..> / start / stop\r\n");
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
    const AlphaBeta v{amp * cosf(thetaE), amp * sinf(thetaE)};
    d = spwm(v, vdc);
    pwm.setDuty(d);
  }

  float ia, ib, ic;
  cs.read(ia, ib, ic);

  telemetry::set(0, ia);
  telemetry::set(1, d.a);
  telemetry::set(2, d.b);
  telemetry::set(3, d.c);
  telemetry::set(4, amp);
  telemetry::set(5, thetaE);
  telemetry::tick();
}
