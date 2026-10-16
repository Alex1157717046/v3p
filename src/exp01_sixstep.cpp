/**
 * E1.1 从零实现六步换相（阶段 1，开环）
 *
 * 目的：理解换相、反 EMF，以及"为什么六步不够好"。
 *
 * 实现要点：
 *   1. 电角度 θe = p·θm，按 π/3 划分 6 个扇区，每 60° 换相一次
 *   2. 每个扇区：一相上管常通、一相下管常通、第三相悬空
 *   3. 悬空相必须斩波到 duty=0.5，使其平均电压 = 0（否则相当于额外加了电压）
 *   4. ★ 本板只有 3 根 PWM 线（FD6287T 的 HIN 与 LIN# 并联），
 *      duty=1（上管常通）时自举电容会持续放电 —— 长时间 100% 占空比会掉驱动，
 *      这正是 E1.7③ 要观察的现象。所以这里用"斩波式六步"：
 *        上管相 duty = scale（默认 0.3，避免大电流/自举掉电）
 *        下管相 duty = 0
 *        悬空相 duty = 0.5
 *      线电压 = scale × Vdc，改 scale 就能限制电流。
 *
 * 串口命令：
 *   f <Hz>        电频率
 *   scale <0~1>   斩波占空比（限制电压/电流）
 *   start / stop
 *   en 0|1        手动使能
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static PwmDriver    pwm;
static CurrentSense cs;
static Console      con;

static bool  running = false;
static float fe      = 1.0f;    // 电频率 Hz
static float scale   = 0.30f;   // 斩波占空比
static float thetaE  = 0.0f;
static uint32_t tPrev = 0;

static void applySixStep() {
  const int step = (int)(thetaE / (PI_F / 3.0f)) % 6;
  float a, b, c;
  sixStep(step, &a, &b, &c);
  // 把"1"（上管常通）换成 scale 斩波，保留 0 与 0.5
  auto chop = [](float d) {
    if (d > 0.75f) return scale;   // 原 1.0 → 斩波
    return d;                       // 0.0 / 0.5 保持不变
  };
  pwm.setDuty(chop(a), chop(b), chop(c));
}

static void cmd_f(int argc, char** argv) {
  fe = Console::argf(argv, argc, 1, fe);
  if (fe < 0) fe = 0;
  Console::printfln("fe = %.3f Hz  (机械转速 %.1f rpm @ p=%d)", fe, fe * 60.0f / POLE_PAIRS_DEFAULT,
                    POLE_PAIRS_DEFAULT);
}

static void cmd_scale(int argc, char** argv) {
  scale = Console::argf(argv, argc, 1, scale);
  scale = fmaxf(0.0f, fminf(1.0f, scale));
  Console::printfln("scale = %.3f  → 线电压峰值约 %.2f V", scale, scale * VDC_DEFAULT);
}

static void cmd_start(int, char**) {
  thetaE = 0.0f;
  tPrev  = micros();
  pwm.enable(true);
  running = true;
  Console::print("six-step RUNNING\r\n");
}

static void cmd_stop(int, char**) {
  running = false;
  pwm.coast();
  Console::print("stopped\r\n");
}

static void cmd_en(int argc, char** argv) {
  pwm.enable(Console::argi(argv, argc, 1, 0) != 0);
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);

  Console::print("\r\n=== E1.1 六步换相（开环）===\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(20000.0f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(512);

  // 上电先给 50/50/50（零电压）再使能，不会有冲击；此处保持失能，等 start
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  con.add("f",     cmd_f,     "f <Hz> 电频率");
  con.add("scale", cmd_scale, "scale <0~1> 斩波占空比");
  con.add("start", cmd_start, "开始");
  con.add("stop",  cmd_stop,  "停止");
  con.add("en",    cmd_en,    "en 0|1");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "i_b");
  telemetry::setChannelName(2, "i_c");
  telemetry::setChannelName(3, "theta_e");
  telemetry::setChannelName(4, "step");
  telemetry::setChannelName(5, "fe");

  Console::print("命令：f <Hz> / scale <0~1> / start / stop\r\n");
}

void loop() {
  con.poll();

  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;      // 防止异常大跳

  if (running) {
    thetaE = wrap2Pi(thetaE + TWO_PI_F * fe * dt);
    applySixStep();
  }

  float ia, ib, ic;
  cs.read(ia, ib, ic);

  telemetry::set(0, ia);
  telemetry::set(1, ib);
  telemetry::set(2, ic);
  telemetry::set(3, thetaE);
  telemetry::set(4, (float)((int)(thetaE / (PI_F / 3.0f)) % 6));
  telemetry::set(5, fe);
  telemetry::tick();
}
