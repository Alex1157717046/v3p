/**
 * E4.3 滑模观测器（SMO）（阶段 4）
 *
 * 目标：实现 SMO，观察抖振与抑制。
 * 验收：抖振显著降低；ê 与 E4.1 实测的反 EMF 吻合。
 *
 * 原理（电流滑模观测器）：
 *   dî/dt = (v − R·i − z)/L ,  z = K·f(î − i)
 *   收敛后 z ≈ 反 EMF e →  θ̂ = atan2(−êα, êβ)
 *   抖振抑制手段：sign → sigmoid/sat（边界层）、对 z 再加低通
 *
 * 串口命令（另见 obs_common.h）：
 *   fn sign|sigmoid|sat
 *   k <v>      滑模增益
 *   a <v>      边界层宽度
 *   lpf <Hz>   提取反 EMF 的低通
 *   pll 0|1    角度直接用 atan2 还是过 PLL
 *   report / sweepspd
 */

#include <Arduino.h>
#include "obs_common.h"

static void cmdFn(int argc, char** argv) {
  if (argc >= 2) {
    if      (!strcmp(argv[1], "sign"))    g_obs.smoFn = SmoFn::Sign;
    else if (!strcmp(argv[1], "sigmoid")) g_obs.smoFn = SmoFn::Sigmoid;
    else if (!strcmp(argv[1], "sat"))     g_obs.smoFn = SmoFn::Sat;
  }
  const char* n = (g_obs.smoFn == SmoFn::Sign) ? "sign(抖振最大)"
                : (g_obs.smoFn == SmoFn::Sat) ? "sat" : "sigmoid(最平滑)";
  Console::printfln("切换函数 = %s", n);
  g_obs.reset();
  obsClearErr();
}

static void cmdK(int argc, char** argv) {
  g_obs.smoK = Console::argf(argv, argc, 1, g_obs.smoK);
  Console::printfln("滑模增益 K = %.2f", g_obs.smoK);
}
static void cmdA(int argc, char** argv) {
  g_obs.smoA = Console::argf(argv, argc, 1, g_obs.smoA);
  Console::printfln("边界层宽度 a = %.4f", g_obs.smoA);
}
static void cmdLpf(int argc, char** argv) {
  g_obs.smoLpfFc = Console::argf(argv, argc, 1, g_obs.smoLpfFc);
  Console::printfln("反 EMF 提取低通 fc = %.1f Hz", g_obs.smoLpfFc);
}
static void cmdPll(int argc, char** argv) {
  g_obs.usePll = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("角度输出 = %s", g_obs.usePll ? "PLL" : "atan2 直算");
  obsClearErr();
}

/// 抖振定量：连续统计 |ω̂| 的波动（抖振大 → 波动大）
static void cmdChatter(int argc, char** argv) {
  const float sec = Console::argf(argv, argc, 1, 1.0f);
  const uint32_t t0 = millis();
  double s = 0, s2 = 0; int n = 0;
  while (millis() - t0 < (uint32_t)(sec * 1000.0f)) {
    const float w = g_obs.omega();
    s += w; s2 += (double)w * w; n++;
    delayMicroseconds(500);
  }
  const double m = s / n, v = s2 / n - m * m;
  Console::printfln("ω̂e 统计：均值 %.1f rad/s，波动(RMS) %.1f rad/s（越小说明抖振越小）", (float)m,
                    (float)sqrt(v > 0 ? v : 0));
}

void setup() {
  obsBegin();
  g_obs.type = ObserverType::Smo;
  g_obs.designPll(TWO_PI_F * 50.0f, 1.0f);

  Console::print("\r\n=== E4.3 滑模观测器（SMO）===\r\n");
  Console::print("流程：align → start → spd 20 → fn sign → chatter → fn sigmoid → chatter\r\n");

  g_con.add("fn",      cmdFn,      "fn sign|sigmoid|sat");
  g_con.add("k",       cmdK,       "k <滑模增益>");
  g_con.add("a",       cmdA,       "a <边界层宽度>");
  g_con.add("lpf",     cmdLpf,     "lpf <Hz>");
  g_con.add("pll",     cmdPll,     "pll 0|1");
  g_con.add("chatter", cmdChatter, "chatter <sec> 统计抖振");
  obsAddTelemetryNames();
}

void loop() { obsLoop(); }
