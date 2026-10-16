/**
 * E1.4 五段式 / DPWM 与纹波对比（阶段 1，开环）
 *
 * 目的：实测"开关次数 ↓33% vs 纹波 ↑"的权衡。
 *
 * 实现：五段式 = 七段式刨掉公共零矢量（svpwm5），等价于把 T0 集中到周期一端。
 *      线电压不变，共模改变 → 开关次数从 6 次/周期降到 4 次/周期。
 *
 * 软件里的"开关次数估计"：duty 落在 0 或 1 的相在本周期不开关，
 * 其余相每周期开关 2 次 → switches = 2 × (未钳位相数)。
 *   SVPWM7  → 6 次/周期
 *   SVPWM5  → 4 次/周期
 *   SPWM    → 6 次/周期
 * （示波器上直接数栅极波形的边沿即可验证）
 *
 * ★ 云台电机电感大（~10 mH），20 kHz 下电流纹波只有 ~10 mA，肉眼难辨。
 *   本实验必须把 fsw 降到 1~2 kHz 才看得见（见 12.4 / 附录 C 结论 1）。
 *   这也是"低载波比"实验共用的手段。
 *
 * 串口命令：f / m / fsw <Hz> / mode spwm|svpwm7|svpwm5 / start / stop
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
static float fsw  = 20000.0f;
static Mod   mode = Mod::SVPWM5;
static float thetaE = 0.0f;
static uint32_t tPrev = 0;

static void cmd_f(int argc, char** argv)   { fe = Console::argf(argv, argc, 1, fe); }
static void cmd_m(int argc, char** argv)   { m = Console::argf(argv, argc, 1, m); }
static void cmd_vdc(int argc, char** argv) { vdc = Console::argf(argv, argc, 1, vdc); }

static void cmd_fsw(int argc, char** argv) {
  fsw = Console::argf(argv, argc, 1, fsw);
  pwm.setFrequency(fsw);
  // 载波比 N = fsw / fe
  Console::printfln("fsw = %.0f Hz  载波比 N = %.1f  (fe=%.3f Hz)", fsw, fsw / fmaxf(fe, 1e-3f), fe);
}

static void cmd_mode(int argc, char** argv) {
  if (argc < 2) { Console::printfln("mode = %s", modName(mode)); return; }
  if      (!strcmp(argv[1], "spwm"))   mode = Mod::SPWM;
  else if (!strcmp(argv[1], "svpwm7")) mode = Mod::SVPWM7;
  else if (!strcmp(argv[1], "svpwm5")) mode = Mod::SVPWM5;
  else { Console::print("unknown mode\r\n"); return; }
  Console::printfln("mode = %s", modName(mode));
}

static void cmd_start(int, char**) { thetaE = 0; tPrev = micros(); pwm.enable(true); running = true; }
static void cmd_stop(int, char**)  { running = false; pwm.coast(); }

/// 统计一个电周期内的平均开关次数（软件估计，见文件头注释）
static int countSwitches(const Duty& d) {
  int n = 0;
  const float eps = 1e-3f;
  const float dd[3] = {d.a, d.b, d.c};
  for (int i = 0; i < 3; i++) {
    if (dd[i] > eps && dd[i] < 1.0f - eps) n += 2;
  }
  return n;
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E1.4 五段式/DPWM 与纹波对比 ===\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(fsw);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(512);

  con.add("f",     cmd_f,     "f <Hz>");
  con.add("m",     cmd_m,     "m <0~1>");
  con.add("vdc",   cmd_vdc,   "vdc <V>");
  con.add("fsw",   cmd_fsw,   "fsw <Hz> 改载波频率（低载波比实验共用）");
  con.add("mode",  cmd_mode,  "mode spwm|svpwm7|svpwm5");
  con.add("start", cmd_start, "开始");
  con.add("stop",  cmd_stop,  "停止");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "duty_a");
  telemetry::setChannelName(2, "duty_b");
  telemetry::setChannelName(3, "duty_c");
  telemetry::setChannelName(4, "sw_per_period");
  telemetry::setChannelName(5, "carrier_ratio");

  Console::print("命令：fsw <Hz> / mode ... / m <..> / start\r\n");
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
    d = modulate(mode, v, vdc);
    clampDuty(d);
    pwm.setDuty(d);
  }

  float ia, ib, ic;
  cs.read(ia, ib, ic);
  telemetry::set(0, ia);
  telemetry::set(1, d.a);
  telemetry::set(2, d.b);
  telemetry::set(3, d.c);
  telemetry::set(4, (float)countSwitches(d));
  telemetry::set(5, pwm.frequency() / fmaxf(fe, 1e-3f));
  telemetry::tick();
}
