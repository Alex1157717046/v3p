/**
 * E3.1 电流环（d 轴）★ 验证 Kp = ωc·L、Ki = ωc·R（阶段 3）
 *
 * 目标：亲手整定电流环，验证理论公式。
 * 验收：实测带宽接近设计值（±20%）；超调 < 20%（ζ≈0.7）；相位裕度 > 45°（用超调反推）。
 *
 * ⚠ 做本实验请把转子【锁住】（否则 d 轴电流也会带动转子）：
 *   手按住 / 用联轴器固定 / 用台钳夹住转轴。
 *
 * 步骤：
 *   ① 先在 E5.1/E5.2 测 L、R（或用本工程默认量级估算 10 Ω / 10 mH）
 *   ② `wc 300` → 自动按 Kp=ωc·L、Ki=ωc·R 算增益
 *   ③ `step 0.3` → 施加 d 轴电流阶跃，自动测上升时间/超调/带宽
 *   ④ 改 ωc 反复做，找到"开始振荡"的 ωc
 *
 * 串口命令：
 *   align           对齐并把编码器清零（电机需可自由转动）
 *   wc <Hz>         设计电流环带宽
 *   kp <v> / ki <v> 手动改增益
 *   axis d|q        阶跃作用在哪个轴
 *   step <A>        施加阶跃并开始测量
 *   en 0|1          使能
 *   report          打印指标
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static FocMotor    m;
static Console     con;
static StepMetrics sm;

static float wcHz  = 300.0f;
static float stepA = 0.30f;
static char  axis  = 'd';
static bool  running = false;
static bool  measuring = false;
static float tSec = 0.0f;
static uint32_t tBase = 0;

static void cmd_align(int, char**) {
  Console::print("aligning...\r\n");
  m.alignSensor(1.0f, 1500);
  Console::printfln("done. θm=%.4f rad", m.enc.angle());
}

static void cmd_wc(int argc, char** argv) {
  wcHz = Console::argf(argv, argc, 1, wcHz);
  m.designCurrentLoop(TWO_PI_F * wcHz);
  Console::printfln("ωc = %.0f Hz  →  Kp = %.4f V/A  Ki = %.1f V/(A·s)", wcHz, m.pidD.kp, m.pidD.ki);
  Console::printfln("（Kp=ωc·L=%.1f×%.4f，Ki=ωc·R=%.1f×%.1f）", TWO_PI_F * wcHz, m.cfg.ls,
                    TWO_PI_F * wcHz, m.cfg.rs);
}

static void cmd_kp(int argc, char** argv) {
  m.pidD.kp = m.pidQ.kp = Console::argf(argv, argc, 1, m.pidD.kp);
  Console::printfln("Kp = %.4f", m.pidD.kp);
}

static void cmd_ki(int argc, char** argv) {
  m.pidD.ki = m.pidQ.ki = Console::argf(argv, argc, 1, m.pidD.ki);
  Console::printfln("Ki = %.1f", m.pidD.ki);
}

static void cmd_axis(int argc, char** argv) {
  if (argc >= 2) axis = argv[1][0];
  Console::printfln("axis = %c", axis);
}

static void cmd_step(int argc, char** argv) {
  stepA = Console::argf(argv, argc, 1, stepA);
  const float fb = (axis == 'd') ? m.id : m.iq;
  sm.start(stepA, fb, tSec);
  measuring = true;
  Console::printfln("阶跃 %.3f A（%c 轴），当前 %.3f A", stepA, axis, fb);
}

static void cmd_en(int argc, char** argv) {
  running = Console::argi(argv, argc, 1, 0) != 0;
  if (running) { tBase = micros(); tSec = 0.0f; }
  m.enable(running);
  Console::printfln("driver %s", running ? "ENABLED" : "disabled");
}

static void cmd_report(int, char**) {
  Console::print("---- 阶跃响应指标 ----\r\n");
  Console::printfln("ωc 设计 = %.0f Hz   Kp=%.4f  Ki=%.1f", wcHz, m.pidD.kp, m.pidD.ki);
  Console::printfln("上升时间(10-90%%) = %.2f ms", sm.riseTime() * 1000.0f);
  Console::printfln("估算带宽 0.35/tr = %.1f Hz   (设计 %.0f Hz)", sm.bandwidthHz(), wcHz);
  Console::printfln("超调量 = %.1f %%   <- 验收 < 20%%（ζ≈0.7 时约 5%%）", sm.overshootPct());
  Console::printfln("调节时间(±2%%) = %.1f ms", sm.settlingTime() * 1000.0f);
  Console::printfln("稳态误差 = %+.4f A", sm.steadyError());
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E3.1 电流环 d 轴：Kp=ωc·L, Ki=ωc·R ===\r\n");
  Console::print("★ 请先锁住转子；流程：align → wc 300 → en 1 → step 0.3 → report\r\n");

  FocConfig c;
  c.fsw = 20000.0f;
  m.begin(c);
  m.setLoopHz(5000.0f);
  cmd_wc(0, nullptr);

  con.add("align",  cmd_align,  "对齐并清零编码器");
  con.add("wc",     cmd_wc,     "wc <Hz> 设计带宽");
  con.add("kp",     cmd_kp,     "kp <V/A>");
  con.add("ki",     cmd_ki,     "ki <V/(A·s)>");
  con.add("axis",   cmd_axis,   "axis d|q");
  con.add("step",   cmd_step,   "step <A> 阶跃并测量");
  con.add("en",     cmd_en,     "en 0|1");
  con.add("report", cmd_report, "打印指标");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_ref");
  telemetry::setChannelName(1, "id");
  telemetry::setChannelName(2, "iq");
  telemetry::setChannelName(3, "vd");
  telemetry::setChannelName(4, "vq");
  telemetry::setChannelName(5, "v_limit");
}

void loop() {
  con.poll();

  if (tBase == 0) tBase = micros();
  tSec = (float)(micros() - tBase) * 1e-6f;

  if (running) {
    if (axis == 'd') m.setCurrentRef(measuring ? stepA : 0.0f, 0.0f);
    else             m.setCurrentRef(0.0f, measuring ? stepA : 0.0f);
  } else {
    m.setCurrentRef(0.0f, 0.0f);
    measuring = false;
  }

  m.update();

  if (measuring) {
    sm.feed((axis == 'd') ? m.id : m.iq, tSec);
    if (sm.done()) {
      measuring = false;
      cmd_report(0, nullptr);
    }
  }

  telemetry::set(0, (axis == 'd') ? (measuring ? stepA : 0.0f) : 0.0f);
  telemetry::set(1, m.id);
  telemetry::set(2, m.iq);
  telemetry::set(3, m.vd);
  telemetry::set(4, m.vq);
  telemetry::set(5, m.vLimitScale);
  telemetry::tick();
}
