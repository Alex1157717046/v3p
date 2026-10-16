/**
 * E3.6 滤波器与相位代价（阶段 3）
 *
 * 目标：验证"滤波器吃掉带宽"。
 * 验收：复现"截止频率 < 3× 控制带宽时相位裕度急剧恶化"。
 *
 * 做法：速度环反馈通路上加一阶低通，扫它的截止频率，同时测：
 *   · 速度噪声（std，反映滤波效果）
 *   · 阶跃超调/振荡（反映相位裕度损失）
 *   → 画出"噪声 vs 相位裕度"的权衡曲线
 *
 * 串口命令：
 *   align / start / stop
 *   lpf <Hz>        速度反馈低通截止频率（0 = 直通）
 *   wcs <Hz>        速度环带宽
 *   stat <sec>      测当前速度噪声 std
 *   step <rad/s>    阶跃并测超调
 *   compare         自动扫截止频率，打印权衡表
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static FocMotor    m;
static Console     con;
static PiController pidSpeed;
static LowPass     velLpf;
static StepMetrics sm;

static bool  running = false;
static float spdRef = 0.0f;
static float lpfHz  = 0.0f;
static float wcsHz  = 10.0f;
static bool  measuring = false;
static float tSec = 0.0f;
static uint32_t tBase = 0;
static float velNoise = 0.0f;
static float lastOver = 0.0f;

static void designSpeedLoop(float hz) {
  const float wc = TWO_PI_F * hz;
  pidSpeed.kp = wc * 2.0e-6f / 0.083f;
  pidSpeed.ki = wc * pidSpeed.kp * 0.5f;
  pidSpeed.outMin = -m.cfg.currentLimit;
  pidSpeed.outMax =  m.cfg.currentLimit;
  pidSpeed.intMin = pidSpeed.outMin;
  pidSpeed.intMax = pidSpeed.outMax;
}

static void cmd_align(int, char**) {
  m.alignSensor(1.0f, 1500);
  Console::print("aligned\r\n");
}

static void cmd_lpf(int argc, char** argv) {
  lpfHz = Console::argf(argv, argc, 1, lpfHz);
  velLpf.setCutoff(lpfHz, 5000.0f);
  velLpf.reset(m.enc.velocity());
  if (lpfHz > 0.0f) {
    Console::printfln("速度反馈低通 fc = %.1f Hz（在 %.1f Hz 控制带宽下的相位滞后 = %.1f°）",
                      lpfHz, wcsHz, LowPass::phaseLagDeg(lpfHz, wcsHz));
  } else {
    Console::print("速度反馈低通：直通（无滤波）\r\n");
  }
}

static void cmd_wcs(int argc, char** argv) {
  wcsHz = Console::argf(argv, argc, 1, wcsHz);
  designSpeedLoop(wcsHz);
  Console::printfln("速度环 ωc=%.1f Hz → Kp=%.5f Ki=%.4f", wcsHz, pidSpeed.kp, pidSpeed.ki);
}

static void cmd_start(int, char**) { pidSpeed.reset(); m.enable(true); running = true; }
static void cmd_stop(int, char**)  { running = false; m.enable(false); }

static void cmd_spd(int argc, char** argv) {
  spdRef = Console::argf(argv, argc, 1, spdRef);
  Console::printfln("ω_ref = %.1f rad/s", spdRef);
}

/// 测速度噪声（在恒定转速下，噪声主要来自编码器量化与差分）
static void cmd_stat(int argc, char** argv) {
  const float sec = Console::argf(argv, argc, 1, 2.0f);
  const uint32_t t0 = millis();
  double s = 0, s2 = 0;
  int n = 0;
  while (millis() - t0 < (uint32_t)(sec * 1000.0f)) {
    const float v = velLpf.value();
    s += v; s2 += (double)v * v; n++;
    delayMicroseconds(200);
  }
  const double mean = s / n;
  const double var = s2 / n - mean * mean;
  velNoise = (float)sqrt(var > 0 ? var : 0);
  Console::printfln("速度噪声 std = %.4f rad/s (%.2f rpm) @ fc=%.1f Hz, ω=%.1f rad/s", velNoise,
                    velNoise * 60.0f / TWO_PI_F, lpfHz, (float)mean);
}

static void cmd_step(int argc, char** argv) {
  spdRef = Console::argf(argv, argc, 1, 30.0f);
  tBase = micros(); tSec = 0.0f;
  sm.start(spdRef, velLpf.value(), tSec);
  measuring = true;
}

static void cmd_report(int, char**) {
  Console::printfln("fc=%.1f Hz  ωc=%.1f Hz  相位滞后=%.1f°  噪声=%.4f rad/s  超调=%.1f%%",
                    lpfHz, wcsHz, LowPass::phaseLagDeg(lpfHz, wcsHz), velNoise, lastOver);
}

/// 扫截止频率：噪声 vs 超调
static void cmd_compare(int, char**) {
  Console::printfln("---- 滤波器截止频率扫描（速度环 ωc=%.1f Hz）----", wcsHz);
  Console::print("fc(Hz)   滞后(deg)   噪声(rad/s)   超调(%)   判定\r\n");
  const float list[] = {0, 200, 100, 50, 30, 20, 10, 5};
  for (float fc : list) {
    lpfHz = fc;
    velLpf.setCutoff(fc, 5000.0f);
    pidSpeed.reset();
    spdRef = 30.0f;
    delay(1200);
    // 测噪声
    {
      const uint32_t t0 = millis();
      double s = 0, s2 = 0; int n = 0;
      while (millis() - t0 < 500) {
        const float v = velLpf.value();
        s += v; s2 += (double)v * v; n++;
        delayMicroseconds(200);
      }
      const double mean = s / n, var = s2 / n - mean * mean;
      velNoise = (float)sqrt(var > 0 ? var : 0);
    }
    // 测阶跃
    tBase = micros(); tSec = 0.0f;
    sm.start(60.0f, velLpf.value(), tSec);
    const uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      tSec = (float)(micros() - tBase) * 1e-6f;
      const float wm = velLpf.update(m.enc.velocity());
      const float iq = pidSpeed.update(spdRef - wm, 0.001f);
      m.setCurrentRef(0.0f, iq);
      m.update();
      sm.feed(velLpf.value(), tSec);
      if (sm.done()) break;
    }
    lastOver = sm.overshootPct();
    const float lag = LowPass::phaseLagDeg(fc, wcsHz);
    const char* verdict = (lastOver > 40.0f) ? "★相位裕度不足" :
                          (lastOver > 20.0f) ? "偏大" : "合适";
    Console::printfln("%6.0f   %8.1f   %10.4f   %8.1f   %s", fc, lag, velNoise, lastOver, verdict);
    spdRef = 0.0f;
    delay(300);
  }
  Console::print("→ 期望看到：fc 降到 3×ωc 以下时超调急剧上升（相位裕度被吃掉）\r\n");
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E3.6 滤波器与相位代价 ===\r\n");

  FocConfig c;
  c.fsw = 20000.0f;
  c.currentLimit = 1.0f;
  m.begin(c);
  m.setLoopHz(5000.0f);
  m.designCurrentLoop(TWO_PI_F * 300.0f);
  // 本实验要自己控制滤波，所以把编码器内部低通关掉（=直通）
  m.enc.setVelocityLpf(0.0f, 1000.0f);
  designSpeedLoop(wcsHz);
  velLpf.setCutoff(0.0f, 5000.0f);

  con.add("align",   cmd_align,   "对齐");
  con.add("start",   cmd_start,   "启动");
  con.add("stop",    cmd_stop,    "停止");
  con.add("spd",     cmd_spd,     "spd <rad/s>");
  con.add("lpf",     cmd_lpf,     "lpf <Hz> 反馈低通");
  con.add("wcs",     cmd_wcs,     "wcs <Hz> 速度环带宽");
  con.add("stat",    cmd_stat,    "stat <sec> 测速度噪声");
  con.add("step",    cmd_step,    "step <rad/s>");
  con.add("report",  cmd_report,  "打印当前指标");
  con.add("compare", cmd_compare, "自动扫截止频率");

  telemetry::setChannels(5);
  telemetry::setChannelName(0, "omega_ref");
  telemetry::setChannelName(1, "omega_raw");
  telemetry::setChannelName(2, "omega_filt");
  telemetry::setChannelName(3, "iq");
  telemetry::setChannelName(4, "lpf_hz");

  Console::print("流程：align → wcs 10 → start → lpf 100 → stat 2 → compare\r\n");
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

  const float vRaw = m.enc.velocity();
  const float vFilt = velLpf.update(vRaw);

  float iqRef = 0.0f;
  if (running) {
    iqRef = pidSpeed.update(spdRef - vFilt, dt);
    m.setCurrentRef(0.0f, iqRef);
  } else {
    m.setCurrentRef(0.0f, 0.0f);
  }

  m.update();

  if (measuring) {
    sm.feed(vFilt, tSec);
    if (sm.done()) { measuring = false; lastOver = sm.overshootPct(); cmd_report(0, nullptr); }
  }

  telemetry::set(0, spdRef);
  telemetry::set(1, vRaw);
  telemetry::set(2, vFilt);
  telemetry::set(3, m.iq);
  telemetry::set(4, lpfHz);
  telemetry::tick();
}
