/**
 * E3.2 电流环（q 轴）+ 解耦前馈（阶段 3）
 *
 * 目标：验证"交叉耦合 + 反 EMF 前馈"的效果。
 * 验收：开解耦后 id 的扰动显著减小；iq 跟踪更快。
 *
 * 原理（表贴式 PMSM，id=0 控制）：
 *   vd = R·id + L·did/dt − ωe·L·iq
 *   vq = R·iq + L·diq/dt + ωe·L·id + ωe·λm
 *                         ↑ 交叉耦合      ↑ 反 EMF
 * 只靠 PI 也能稳态无差，但动态时 id 会被 ωe·L·iq 顶出一个尖峰 —— 这就是要观测的现象。
 *
 * ⚠ 本实验需要电机【能转】（要有转速才有耦合），务必固定好台架。
 *
 * 串口命令：
 *   align            对齐
 *   start / stop
 *   spd <rad/s>      速度环给定（外环很慢，只为了让电机转起来）
 *   spdstep <rad/s>  速度阶跃，观察 id 扰动
 *   decouple 0|1     交叉耦合前馈
 *   bef 0|1          反 EMF 前馈
 *   clear            清除 id 扰动峰值
 *   report           打印 id 扰动峰值对比
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static FocMotor    m;
static Console     con;
static PiController pidSpeed;

static bool  running = false;
static float spdRef  = 0.0f;      // rad/s（机械）
static float wcsHz   = 5.0f;      // 速度环带宽
static float idPeak  = 0.0f;
static float idPeakNoDec = 0.0f;
static bool  lastDecouple = false;

static void cmd_align(int, char**) {
  Console::print("aligning...\r\n");
  m.alignSensor(1.0f, 1500);
  m.enc.setZero();
  Console::print("done\r\n");
}

static void cmd_start(int, char**) {
  pidSpeed.reset();
  m.enable(true);
  running = true;
  idPeak = 0.0f;
  Console::print("running\r\n");
}
static void cmd_stop(int, char**) { running = false; m.enable(false); }

static void cmd_spd(int argc, char** argv) {
  spdRef = Console::argf(argv, argc, 1, spdRef);
  idPeak = 0.0f;
  Console::printfln("ω_ref = %.2f rad/s (%.0f rpm)", spdRef, spdRef * 60.0f / TWO_PI_F);
}

static void cmd_spdstep(int argc, char** argv) {
  spdRef = Console::argf(argv, argc, 1, spdRef);
  idPeak = 0.0f;
  Console::printfln("速度阶跃到 %.2f rad/s，开始记录 id 扰动峰值", spdRef);
}

static void cmd_decouple(int argc, char** argv) {
  if (m.decouple) idPeakNoDec = fmaxf(idPeakNoDec, idPeak);
  m.decouple = Console::argi(argv, argc, 1, 1) != 0;
  lastDecouple = m.decouple;
  idPeak = 0.0f;
  Console::printfln("解耦前馈 %s", m.decouple ? "ON" : "OFF");
}

static void cmd_bef(int argc, char** argv) {
  m.backEmfFeedforward = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("反 EMF 前馈 %s", m.backEmfFeedforward ? "ON" : "OFF");
}

static void cmd_wcs(int argc, char** argv) {
  wcsHz = Console::argf(argv, argc, 1, wcsHz);
  // 简化设计：J 未知，用"速度环带宽 ≤ 电流环/5"的原则给一组可用初值
  pidSpeed.kp = TWO_PI_F * wcsHz * 0.002f / 0.083f;   // ωc·J/Kt（J≈2e-6, Kt≈0.083）
  pidSpeed.ki = pidSpeed.kp * TWO_PI_F * wcsHz * 0.3f;
  pidSpeed.outMin = -m.cfg.currentLimit;
  pidSpeed.outMax =  m.cfg.currentLimit;
  pidSpeed.intMin = pidSpeed.outMin;
  pidSpeed.intMax = pidSpeed.outMax;
  Console::printfln("速度环 ωc=%.1f Hz → Kp=%.4f Ki=%.3f", wcsHz, pidSpeed.kp, pidSpeed.ki);
}

static void cmd_clear(int, char**) { idPeak = 0.0f; }
static void cmd_ceiling(int, char**) { idPeakNoDec = 0.0f; Console::print("清空对比记录\r\n"); }

static void cmd_report(int, char**) {
  Console::print("---- 交叉耦合（id 扰动）对比 ----\r\n");
  Console::printfln("当前 decouple=%d bef=%d, ω_ref=%.2f rad/s", m.decouple ? 1 : 0,
                    m.backEmfFeedforward ? 1 : 0, spdRef);
  Console::printfln("本次 id 扰动峰值 = %.4f A（id_ref=0，理想应为 0）", idPeak);
  Console::printfln("历史最大（关解耦时）= %.4f A", idPeakNoDec);
  Console::printfln("提示：先 decouple 0 → spdstep 30 → 记下峰值；再 decouple 1 → spdstep 30 → 对比");
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E3.2 q 轴电流环 + 解耦前馈 ===\r\n");

  FocConfig c;
  c.fsw = 20000.0f;
  c.currentLimit = 1.0f;
  m.begin(c);
  m.setLoopHz(5000.0f);
  m.designCurrentLoop(TWO_PI_F * 300.0f);
  cmd_wcs(0, nullptr);

  con.add("align",    cmd_align,    "对齐");
  con.add("start",    cmd_start,    "启动");
  con.add("stop",     cmd_stop,     "停止");
  con.add("spd",      cmd_spd,      "spd <rad/s>");
  con.add("spdstep",  cmd_spdstep,  "spdstep <rad/s> 速度阶跃并记录");
  con.add("decouple", cmd_decouple, "decouple 0|1");
  con.add("bef",      cmd_bef,      "bef 0|1 反EMF前馈");
  con.add("wcs",      cmd_wcs,      "wcs <Hz> 速度环带宽");
  con.add("clear",    cmd_clear,    "清除本次峰值");
  con.add("ceiling",  cmd_ceiling,  "清空对比记录");
  con.add("report",   cmd_report,   "打印对比");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "id_ref");
  telemetry::setChannelName(1, "id");
  telemetry::setChannelName(2, "iq");
  telemetry::setChannelName(3, "omega_m");
  telemetry::setChannelName(4, "omega_ref");
  telemetry::setChannelName(5, "decouple");
}

void loop() {
  con.poll();

  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  if (running) {
    const float wm = m.enc.velocity();
    const float iqRef = pidSpeed.update(spdRef - wm, dt);
    m.setCurrentRef(0.0f, iqRef);
  } else {
    m.setCurrentRef(0.0f, 0.0f);
  }

  m.update();

  // id 扰动峰值统计（id_ref = 0）
  const float dev = fabsf(m.id);
  if (dev > idPeak) idPeak = dev;

  telemetry::set(0, 0.0f);
  telemetry::set(1, m.id);
  telemetry::set(2, m.iq);
  telemetry::set(3, m.enc.velocity());
  telemetry::set(4, spdRef);
  telemetry::set(5, m.decouple ? 1.0f : 0.0f);
  telemetry::tick();
  (void)lastDecouple;
}
