/**
 * E4.4 PLL 设计与带宽（阶段 4）
 *
 * 目标：实现 PLL，理解 ωn、ζ 的作用。
 * 验收：PLL 输出噪声比 atan2 直算低一个数量级；给出不同 ζ 的对比。
 *
 * ★ 对比点（对应 12.6 E4.4）：VESC 用 ζ=5.77（过阻尼），教科书说 0.707。
 *   亲手试这两种，看角度噪声与动态跟踪的差别 —— 这就是"理解 vs 背过"的分界。
 *
 * 原理：
 *   ε = −(eα·cosθ̂ + eβ·sinθ̂)/|e|  ≈ sin(θ−θ̂) ≈ θ−θ̂
 *   ω̂ = Kp·ε + Ki·∫ε dt ,  θ̂ = ∫ω̂ dt
 *   按二阶系统设计：Kp = 2ζωn，Ki = ωn²
 *
 * 串口命令（另见 obs_common.h）：
 *   pll 0|1        关闭 = 直接用 atan2（对比基线）
 *   wn <Hz>        自然频率
 *   zeta <v>       阻尼比（试 0.707 / 1.0 / 5.77）
 *   report         看误差均值/标准差（对比噪声）
 *   sweepspd
 */

#include <Arduino.h>
#include "obs_common.h"

static float g_wnHz = 50.0f;
static float g_zeta = 0.707f;

static void applyPll() {
  g_obs.designPll(TWO_PI_F * g_wnHz, g_zeta);
  Console::printfln("PLL: ωn = %.1f Hz (%.1f rad/s), ζ = %.3f  →  Kp = %.1f, Ki = %.0f", g_wnHz,
                    TWO_PI_F * g_wnHz, g_zeta, g_obs.pllKp, g_obs.pllKi);
  Console::printfln("（VESC 用的是 ζ=5.77；教科书推荐 0.707。带宽 ≈ ωn·√(2ζ²+√(4ζ⁴+1))）");
  obsClearErr();
}

static void cmdPllOn(int argc, char** argv) {
  g_obs.usePll = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("角度输出 = %s", g_obs.usePll ? "PLL" : "atan2 直算（基线，噪声大）");
  obsClearErr();
}

static void cmdWn(int argc, char** argv) {
  g_wnHz = Console::argf(argv, argc, 1, g_wnHz);
  applyPll();
}

static void cmdZeta(int argc, char** argv) {
  g_zeta = Console::argf(argv, argc, 1, g_zeta);
  applyPll();
}

/// 自动对比几种 ζ
static void cmdCompare(int, char**) {
  Console::printfln("---- PLL 阻尼比对比（ωn=%.0f Hz，转速 %.1f rad/s）----\r\n", g_wnHz, g_spdRef);
  Console::print("  ζ     误差均值(°)  误差std(°)  误差峰值(°)  ω̂e/真值\r\n");
  const float zs[] = {0.3f, 0.707f, 1.0f, 2.0f, 5.77f};
  for (float z : zs) {
    g_zeta = z;
    g_obs.designPll(TWO_PI_F * g_wnHz, z);
    g_obs.reset();
    obsClearErr();
    obsWait(700);
    obsWait(1200);
    if (g_errN < 5) { Console::printfln("%5.2f   —— 样本不足", z); continue; }
    const float mean = g_errSum / g_errN;
    const float var  = g_errSum2 / g_errN - mean * mean;
    Console::printfln("%5.2f  %+11.2f  %10.2f  %11.2f  %8.3f", z, mean,
                      sqrtf(var > 0 ? var : 0), g_errPeak,
                      g_obs.omega() / (fabsf(g_m.omegaE) + 1e-6f));
  }
  Console::print("→ 期望：ζ 小 → 噪声大/易振荡；ζ 大（如 5.77）→ 噪声小但跟踪滞后\r\n");
  g_zeta = 0.707f;
  g_obs.designPll(TWO_PI_F * g_wnHz, g_zeta);
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E4.4 PLL 设计与带宽 ===\r\n");
  Console::print("流程：align → start → spd 20 → pll 0 → report ；pll 1 → report → compare\r\n");
  applyPll();

  g_con.add("pll",     cmdPllOn, "pll 0|1");
  g_con.add("wn",      cmdWn,    "wn <Hz> 自然频率");
  g_con.add("zeta",    cmdZeta,  "zeta <阻尼比>");
  g_con.add("compare", cmdCompare, "自动对比 5 种阻尼比");
  obsAddTelemetryNames();
}

void loop() { obsLoop(); }
