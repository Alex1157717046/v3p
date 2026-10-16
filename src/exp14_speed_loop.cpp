/**
 * E3.3 速度环与带宽分离（阶段 3）
 *
 * 目标：验证"速度环带宽 ≤ 电流环带宽 / 5"。
 * 验收：找到"开始振荡的带宽"，验证它接近电流环带宽的 1/5 量级。
 *
 * 设计公式：Kp,s = ωc,s·J/Kt，Ki,s = ωc,s·Kp,s/若干（本工程用简化整定 + 手动微调）
 * 本工程默认 J ≈ 2e-6 kg·m²（2208 转子），Kt ≈ 0.083 N·m/A（量级估算，E4.1 实测替换）。
 *
 * 串口命令：
 *   align            对齐
 *   wcs <Hz>         设计速度环带宽
 *   spd <rad/s>      目标转速
 *   step <rad/s>     阶跃并测指标
 *   sweep            自动扫带宽：逐个做阶跃、打印超调/上升时间，找到开始振荡的点
 *   start / stop
 *   report
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static FocMotor    m;
static Console     con;
static PiController pidSpeed;
static StepMetrics sm;

static bool  running = false;
static float spdRef = 0.0f;
static float wcsHz  = 5.0f;
static bool  measuring = false;
static uint32_t tBase = 0;
static float tSec = 0.0f;
static float J  = 2.0e-6f;      // kg·m²
static float Kt = 0.083f;       // N·m/A

static void designSpeedLoop(float hz) {
  const float wc = TWO_PI_F * hz;
  pidSpeed.kp = wc * J / Kt;
  pidSpeed.ki = wc * pidSpeed.kp * 0.5f;
  pidSpeed.outMin = -m.cfg.currentLimit;
  pidSpeed.outMax =  m.cfg.currentLimit;
  pidSpeed.intMin = pidSpeed.outMin;
  pidSpeed.intMax = pidSpeed.outMax;
  pidSpeed.aw = AntiWindup::Clamp;
}

static void cmd_align(int, char**) {
  Console::print("aligning...\r\n");
  m.alignSensor(1.0f, 1500);
  m.enc.setZero();
  Console::print("done\r\n");
}

static void cmd_wcs(int argc, char** argv) {
  wcsHz = Console::argf(argv, argc, 1, wcsHz);
  designSpeedLoop(wcsHz);
  Console::printfln("速度环 ωc=%.1f Hz → Kp=%.5f Ki=%.4f", wcsHz, pidSpeed.kp, pidSpeed.ki);
}

static void cmd_spd(int argc, char** argv) {
  spdRef = Console::argf(argv, argc, 1, spdRef);
  Console::printfln("ω_ref = %.2f rad/s (%.0f rpm)", spdRef, spdRef * 60.0f / TWO_PI_F);
}

static void cmd_start(int, char**) {
  pidSpeed.reset();
  m.enable(true);
  running = true;
  tBase = micros();
}
static void cmd_stop(int, char**) { running = false; m.enable(false); }

static void cmd_step(int argc, char** argv) {
  spdRef = Console::argf(argv, argc, 1, 20.0f);
  const float fb = m.enc.velocity();
  tBase = micros(); tSec = 0.0f;
  sm.start(spdRef, fb, tSec);
  measuring = true;
  Console::printfln("速度阶跃 %.2f rad/s（当前 %.2f）", spdRef, fb);
}

static void report_() {
  Console::printfln("ωc=%.1f Hz | tr=%.1f ms | BW≈%.1f Hz | 超调=%.1f%% | 整定=%.0f ms | 稳态误差=%+.3f rad/s",
                    wcsHz, sm.riseTime() * 1000.0f, sm.bandwidthHz(), sm.overshootPct(),
                    sm.settlingTime() * 1000.0f, sm.steadyError());
}

static void cmd_report(int, char**) { report_(); }

/// 自动扫带宽：每个带宽做一次阶跃，打印指标，找"开始振荡"的点
static void cmd_sweep(int, char**) {
  Console::print("速度环带宽扫描（电流环设计 300 Hz，理论上速度环 ≤ 60 Hz）\r\n");
  Console::print("ωc(Hz)   tr(ms)   BW(Hz)   超调(%)   判定\r\n");
  const float list[] = {2, 5, 10, 20, 40, 60, 80, 120};
  for (float hz : list) {
    designSpeedLoop(hz);
    pidSpeed.reset();
    spdRef = 30.0f;
    delay(1500);                       // 先稳到 30 rad/s
    const float fb = m.enc.velocity();
    tBase = micros(); tSec = 0.0f;
    sm.start(60.0f, fb, tSec);
    spdRef = 60.0f;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      tSec = (float)(micros() - tBase) * 1e-6f;
      const float wm = m.enc.velocity();
      const float iq = pidSpeed.update(spdRef - wm, 0.001f);
      m.setCurrentRef(0.0f, iq);
      m.update();
      sm.feed(wm, tSec);
      if (sm.done()) break;
    }
    const char* verdict = (sm.overshootPct() > 40.0f) ? "★开始振荡" :
                          (sm.overshootPct() > 20.0f) ? "偏激进" : "稳定";
    Console::printfln("%6.0f  %7.1f  %7.1f  %8.1f   %s", hz, sm.riseTime() * 1000.0f,
                      sm.bandwidthHz(), sm.overshootPct(), verdict);
  }
  Console::print("→ 与电流环 300 Hz 对比：振荡点是否在 300/5 = 60 Hz 附近？\r\n");
  designSpeedLoop(wcsHz);
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E3.3 速度环与带宽分离 ===\r\n");

  FocConfig c;
  c.fsw = 20000.0f;
  c.currentLimit = 1.0f;
  m.begin(c);
  m.setLoopHz(5000.0f);
  m.designCurrentLoop(TWO_PI_F * 300.0f);
  designSpeedLoop(wcsHz);

  con.add("align",  cmd_align,  "对齐");
  con.add("wcs",    cmd_wcs,    "wcs <Hz>");
  con.add("spd",    cmd_spd,    "spd <rad/s>");
  con.add("step",   cmd_step,   "step <rad/s> 阶跃并测量");
  con.add("sweep",  cmd_sweep,  "自动扫带宽找振荡点");
  con.add("start",  cmd_start,  "启动");
  con.add("stop",   cmd_stop,   "停止");
  con.add("report", cmd_report, "打印指标");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "omega_ref");
  telemetry::setChannelName(1, "omega_m");
  telemetry::setChannelName(2, "iq_ref");
  telemetry::setChannelName(3, "iq");
  telemetry::setChannelName(4, "wcs_hz");
  telemetry::setChannelName(5, "v_limit");

  Console::print("流程：align → wcs 5 → start → step 30 → report ；再 sweep 看振荡点\r\n");
}

void loop() {
  con.poll();

  if (tBase == 0) tBase = micros();
  tSec = (float)(micros() - tBase) * 1e-6f;

  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  float iqRef = 0.0f;
  if (running) {
    const float wm = m.enc.velocity();
    iqRef = pidSpeed.update(spdRef - wm, dt);
    m.setCurrentRef(0.0f, iqRef);
  } else {
    m.setCurrentRef(0.0f, 0.0f);
  }

  m.update();

  if (measuring) {
    sm.feed(m.enc.velocity(), tSec);
    if (sm.done()) { measuring = false; report_(); }
  }

  telemetry::set(0, spdRef);
  telemetry::set(1, m.enc.velocity());
  telemetry::set(2, iqRef);
  telemetry::set(3, m.iq);
  telemetry::set(4, wcsHz);
  telemetry::set(5, m.vLimitScale);
  telemetry::tick();
}
