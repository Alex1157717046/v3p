/**
 * E3.5 抗饱和与限幅（阶段 3）
 *
 * 目标：复现积分饱和现象并修复。
 * 验收：超调从 >50% 降到 <10%；给出三种抗饱和方案的对比表。
 *
 * 做法：故意把电压限幅设得很小（让 PI 输出长时间顶在限幅上），
 *       再施加电流阶跃 → 观察严重超调；然后切换三种方案对比。
 *
 * 三种方案（见 lib/v3plab/v3p_pid.h）：
 *   none     什么都不做（复现饱和）
 *   clamp    条件积分：输出饱和且误差仍推向饱和方向时停止积分
 *   backcalc 反计算：(sat − unsat)·kaw 反馈回积分器
 *
 * 串口命令：
 *   align / start / stop
 *   aw none|clamp|backcalc
 *   vlim <V>     电压限幅（默认 0.3 V，很小 → 一定饱和）
 *   ilim <A>     电流限幅
 *   step <A>     d 轴电流阶跃并测量
 *   compare      自动跑三种方案并打印对比表
 *   report
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static FocMotor    m;
static Console     con;
static StepMetrics sm;

static bool  running = false;
static bool  measuring = false;
static float tSec = 0.0f;
static uint32_t tBase = 0;
static float stepA = 0.5f;
static float vlim  = 0.30f;

static float lastOver = 0.0f, lastTr = 0.0f, lastSs = 0.0f;

static void cmd_align(int, char**) {
  Console::print("aligning...\r\n");
  m.alignSensor(1.0f, 1500);
  Console::print("done\r\n");
}

static void cmd_start(int, char**) {
  m.enable(true);
  running = true;
  tBase = micros();
}
static void cmd_stop(int, char**) { running = false; m.enable(false); }

static void cmd_aw(int argc, char** argv) {
  if (argc < 2) return;
  if      (!strcmp(argv[1], "none"))     m.pidD.aw = m.pidQ.aw = AntiWindup::None;
  else if (!strcmp(argv[1], "clamp"))    m.pidD.aw = m.pidQ.aw = AntiWindup::Clamp;
  else if (!strcmp(argv[1], "backcalc")) m.pidD.aw = m.pidQ.aw = AntiWindup::BackCalc;
  else { Console::print("none|clamp|backcalc\r\n"); return; }
  m.pidD.reset();
  Console::printfln("抗饱和方案 = %s", argv[1]);
}

static void cmd_vlim(int argc, char** argv) {
  vlim = Console::argf(argv, argc, 1, vlim);
  m.setVoltageLimit(vlim);
  m.designCurrentLoop(TWO_PI_F * 300.0f);   // 重新套用限幅到 PI
  m.setVoltageLimit(vlim);
  Console::printfln("电压限幅 = %.3f V（越小越容易饱和）", vlim);
}

static void cmd_ilim(int argc, char** argv) {
  m.cfg.currentLimit = Console::argf(argv, argc, 1, m.cfg.currentLimit);
  Console::printfln("电流限幅 = %.2f A", m.cfg.currentLimit);
}

static void cmd_step(int argc, char** argv) {
  stepA = Console::argf(argv, argc, 1, stepA);
  m.pidD.reset();
  tBase = micros(); tSec = 0.0f;
  sm.start(stepA, m.id, tSec);
  measuring = true;
  Console::printfln("d 轴电流阶跃 %.3f A", stepA);
}

static void report_() {
  lastOver = sm.overshootPct();
  lastTr   = sm.riseTime();
  lastSs   = sm.steadyError();
  const char* name = (m.pidD.aw == AntiWindup::None) ? "none"
                   : (m.pidD.aw == AntiWindup::Clamp) ? "clamp" : "backcalc";
  Console::printfln("[%s] 超调 = %.1f %%   tr = %.1f ms   稳态误差 = %+.4f A", name, lastOver,
                    lastTr * 1000.0f, lastSs);
}

static void cmd_report(int, char**) { report_(); }

/// 自动对比三种方案
static void cmd_compare(int, char**) {
  Console::printfln("---- 抗饱和方案对比（vlim=%.2f V, 阶跃 %.2f A）----", vlim, stepA);
  Console::print("方案        超调(%)    tr(ms)    稳态误差(A)\r\n");
  AntiWindup modes[3] = {AntiWindup::None, AntiWindup::Clamp, AntiWindup::BackCalc};
  const char* names[3] = {"none", "clamp", "backcalc"};
  for (int i = 0; i < 3; i++) {
    m.pidD.aw = m.pidQ.aw = modes[i];
    m.pidD.reset();
    // 先回到零电流
    m.setCurrentRef(0, 0);
    delay(400);
    tBase = micros(); tSec = 0.0f;
    sm.start(stepA, m.id, tSec);
    const uint32_t t0 = millis();
    while (millis() - t0 < 1200) {
      tSec = (float)(micros() - tBase) * 1e-6f;
      m.setCurrentRef(stepA, 0.0f);
      m.update();
      sm.feed(m.id, tSec);
      if (sm.done()) break;
    }
    Console::printfln("%-10s  %7.1f   %7.1f   %+8.4f", names[i], sm.overshootPct(),
                      sm.riseTime() * 1000.0f, sm.steadyError());
    m.setCurrentRef(0, 0);
    delay(300);
  }
  Console::print("→ 期望看到：none 超调很大（>50%），clamp/backcalc 明显减小\r\n");
  m.pidD.aw = m.pidQ.aw = AntiWindup::Clamp;
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E3.5 抗饱和与限幅 ===\r\n");
  Console::print("★ 请先锁住转子（d 轴电流阶跃）；流程：align → vlim 0.3 → start → compare\r\n");

  FocConfig c;
  c.fsw = 20000.0f;
  c.currentLimit = 1.2f;
  m.begin(c);
  m.setLoopHz(5000.0f);
  m.designCurrentLoop(TWO_PI_F * 300.0f);
  m.setVoltageLimit(vlim);

  con.add("align",   cmd_align,   "对齐");
  con.add("start",   cmd_start,   "启动");
  con.add("stop",    cmd_stop,    "停止");
  con.add("aw",      cmd_aw,      "aw none|clamp|backcalc");
  con.add("vlim",    cmd_vlim,    "vlim <V> 电压限幅");
  con.add("ilim",    cmd_ilim,    "ilim <A> 电流限幅");
  con.add("step",    cmd_step,    "step <A>");
  con.add("report",  cmd_report,  "打印上次指标");
  con.add("compare", cmd_compare, "自动对比三种方案");

  telemetry::setChannels(5);
  telemetry::setChannelName(0, "i_ref");
  telemetry::setChannelName(1, "id");
  telemetry::setChannelName(2, "vd");
  telemetry::setChannelName(3, "integral");
  telemetry::setChannelName(4, "v_limit");

  Console::print("命令：vlim 0.3 / start / compare\r\n");
}

void loop() {
  con.poll();

  if (tBase == 0) tBase = micros();
  tSec = (float)(micros() - tBase) * 1e-6f;

  if (running) m.setCurrentRef(measuring ? stepA : 0.0f, 0.0f);
  else         m.setCurrentRef(0.0f, 0.0f);

  m.update();

  if (measuring) {
    sm.feed(m.id, tSec);
    if (sm.done()) { measuring = false; report_(); }
  }

  telemetry::set(0, measuring ? stepA : 0.0f);
  telemetry::set(1, m.id);
  telemetry::set(2, m.vd);
  telemetry::set(3, m.pidD.integral());
  telemetry::set(4, m.vLimitScale);
  telemetry::tick();
}
