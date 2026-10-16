/**
 * E5.6 谐振与陷波（阶段 5）
 *
 * 目标：找到机械谐振并抑制。
 * 验收：谐振峰下降 > 20 dB；速度环带宽可以提得更高。
 *
 * 做法（频响测量）：
 *   速度环正常运行，往 iq_ref 上叠加一个正弦扰动 amp·sin(2πft)，
 *   对速度反馈做【同步检波】得到该频率下的响应幅值 |H(f)|。
 *   扫一遍频率 → 曲线上的峰就是机械谐振点。
 *   然后打开陷波滤波器再做一遍 → 峰应被压下去。
 *
 * ★ 提示：用手捏住电机支架改变刚度，谐振频率会变 → 直接验证"机械谐振"的机理。
 *
 * 串口命令：
 *   f <Hz>          扰动频率
 *   amp <A>         扰动幅值
 *   meas            在当前 f 下测一次响应幅值
 *   sweep           自动扫频（1~200 Hz），打印频响
 *   notch <f0> <bw> 打开陷波
 *   notchoff        关闭陷波
 */

#include <Arduino.h>
#include "obs_common.h"

static NotchFilter g_notch;
static float g_injF = 20.0f;
static float g_injA = 0.10f;
static float g_lastAmp = 0.0f;
static float g_tSec = 0.0f;

static void nServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  const float wRaw = g_m.enc.velocity();
  const float wFb  = g_notch.update(wRaw);

  float iq = 0.0f;
  if (g_running) {
    iq = g_pidSpd.update(g_spdRef - wFb, dt);
    iq += g_injA * sinf(TWO_PI_F * g_injF * g_tSec);   // 叠加扰动
    g_m.setCurrentRef(0.0f, iq);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }
  g_m.update();
}

static void nWait(uint32_t ms, bool sample) {
  const uint32_t t0 = millis();
  double re = 0, im = 0;
  int n = 0;
  while (millis() - t0 < ms) {
    nServiceOnce();
    g_tSec += 1e-3f;      // 近似时基（同步检波对时基误差不敏感，只要频率准确）
    if (sample) {
      const float v = g_m.enc.velocity();
      re += v * cosf(TWO_PI_F * g_injF * g_tSec);
      im += v * sinf(TWO_PI_F * g_injF * g_tSec);
      n++;
    }
    delay(1);
  }
  if (sample && n > 10) {
    g_lastAmp = (float)(2.0 * sqrt(re * re + im * im) / n);
  }
}

static void cmdF(int argc, char** argv) {
  g_injF = Console::argf(argv, argc, 1, g_injF);
  Console::printfln("扰动频率 = %.1f Hz", g_injF);
}
static void cmdAmp(int argc, char** argv) {
  g_injA = Console::argf(argv, argc, 1, g_injA);
  Console::printfln("扰动幅值 = %.3f A", g_injA);
}
static void cmdMeas(int, char**) {
  nWait(1200, false);         // 先过渡
  nWait(1500, true);          // 再同步检波
  Console::printfln("f = %.1f Hz  →  速度响应幅值 = %.5f rad/s   (%.3f rpm)", g_injF, g_lastAmp,
                    g_lastAmp * 60.0f / TWO_PI_F);
  Console::printfln("   增益 |H| = %.4f (rad/s)/A", g_lastAmp / fmaxf(g_injA, 1e-6f));
}
static void cmdNotch(int argc, char** argv) {
  const float f0 = Console::argf(argv, argc, 1, 60.0f);
  const float bw = Console::argf(argv, argc, 2, 10.0f);
  g_notch.design(f0, bw, 1000.0f);     // 速度反馈的采样率约 1 kHz
  g_notch.reset();
  Console::printfln("陷波已开启：f0=%.1f Hz  bw=%.1f Hz", f0, bw);
}
static void cmdNotchOff(int, char**) {
  g_notch.disable();
  Console::print("陷波已关闭\r\n");
}

/// 自动扫频
static void cmdSweep(int, char**) {
  if (!g_running) { Console::print("先 start\r\n"); return; }
  Console::printfln("---- 频响扫描（扰动 %.3f A，陷波 %s）----", g_injA,
                    g_notch.enabled() ? "开" : "关");
  Console::print(" f(Hz)   响应幅值(rad/s)   增益(rad/s)/A    dB\r\n");
  const float fs[] = {1, 2, 5, 10, 20, 30, 40, 50, 60, 80, 100, 150, 200};
  float maxAmp = 1e-9f, maxF = 0;
  for (float f : fs) {
    g_injF = f;
    nWait(900, false);
    nWait(1400, true);
    const float g = g_lastAmp / fmaxf(g_injA, 1e-6f);
    const float db = 20.0f * log10f(fmaxf(g, 1e-9f));
    Console::printfln("%6.0f   %14.5f   %14.4f   %7.1f", f, g_lastAmp, g, db);
    if (g_lastAmp > maxAmp) { maxAmp = g_lastAmp; maxF = f; }
  }
  Console::printfln("→ 峰值出现在 %.0f Hz（谐振点）", maxF);
  Console::printfln("→ 再 notch %.0f 10 后重跑 sweep，比较该点的 dB 是否下降 > 20 dB\r\n", maxF);
  Console::print("→ 用手捏住支架改变刚度，重跑 sweep，谐振频率会移动 —— 这就是机械谐振的直接验证\r\n");
}

static void cmdNStart(int, char**) { cmdStart(0, nullptr); }

void setup() {
  obsBegin();
  Console::print("\r\n=== E5.6 谐振与陷波 ===\r\n");
  Console::print("流程：align → start → spd 30 → sweep → notch <峰频> 10 → sweep\r\n");

  g_con.add("f",        cmdF,        "f <Hz> 扰动频率");
  g_con.add("amp",      cmdAmp,      "amp <A> 扰动幅值");
  g_con.add("meas",     cmdMeas,     "在当前 f 测响应");
  g_con.add("nsweep",   cmdSweep,    "自动扫频");
  g_con.add("notch",    cmdNotch,    "notch <f0> <bw>");
  g_con.add("notchoff", cmdNotchOff, "关闭陷波");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();
  nServiceOnce();
  g_tSec += 1e-3f;

  telemetry::set(0, g_injF);
  telemetry::set(1, g_injA);
  telemetry::set(2, g_lastAmp);
  telemetry::set(3, g_m.enc.velocity());
  telemetry::set(4, g_notch.enabled() ? 1.0f : 0.0f);
  telemetry::set(5, g_m.iq);
  telemetry::tick();
}
