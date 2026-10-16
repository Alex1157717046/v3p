/**
 * E4.2 磁链观测器：从纯积分到可编程 LPF（阶段 4）
 *
 * 目标：亲手复现"纯积分为何发散"及解法。
 * 验收：纯积分漂移明显；换成 ωc 跟随转速的 LPF 后漂移消失；
 *       中高速角度误差 < 5° 电角度。
 *
 * 原理：
 *   λ = ∫(v − R·i)dt                    ← 纯积分，直流偏置会积到饱和
 *   λα = (vα − R·iα)/(s + ωc)           ← 用低通代替纯积分（有泄漏）
 *   若取 ωc = k·|ωe|，则相位滞后 = atan(ωe/ωc) = atan(1/k) 是【常数】
 *   → 直接加一个常数补偿角即可（这就是关键技巧）
 *
 * 串口命令（另见 obs_common.h 的通用命令）：
 *   mode pure|lpf
 *   k <v>          ωc = k·|ωe|
 *   wc0 <rad/s>    最低截止
 *   comp 0|1       常数相位补偿
 *   subli 0|1      取角度前先减 L·i
 *   lam <Wb>       设定 λm（影响 L·i 项）
 *   r <ohm> / l <H>
 *   report / sweepspd
 */

#include <Arduino.h>
#include "obs_common.h"

static void cmdMode(int argc, char** argv) {
  if (argc >= 2) {
    if      (!strcmp(argv[1], "pure")) g_obs.type = ObserverType::FluxPure;
    else if (!strcmp(argv[1], "lpf"))  g_obs.type = ObserverType::FluxLpf;
  }
  Console::printfln("mode = %s", g_obs.type == ObserverType::FluxPure ? "pure(纯积分)" : "lpf(可编程LPF)");
  g_obs.reset();
  obsClearErr();
}

static void cmdK(int argc, char** argv) {
  g_obs.k = Console::argf(argv, argc, 1, g_obs.k);
  Console::printfln("k = %.3f  → 相位滞后 = atan(1/k) = %.2f°（常数）", g_obs.k,
                    atanf(1.0f / fmaxf(g_obs.k, 1e-3f)) * 180.0f / PI_F);
}
static void cmdWc0(int argc, char** argv) { g_obs.wc0 = Console::argf(argv, argc, 1, g_obs.wc0); }
static void cmdComp(int argc, char** argv) {
  g_obs.phaseComp = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("相位补偿 %s", g_obs.phaseComp ? "ON" : "OFF");
}
static void cmdSubLi(int argc, char** argv) {
  g_obs.subtractLi = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("取角度前减 L·i：%s", g_obs.subtractLi ? "ON" : "OFF");
}
static void cmdRL(int argc, char** argv) {
  g_obs.rs = Console::argf(argv, argc, 1, g_obs.rs);
  g_obs.ls = Console::argf(argv, argc, 2, g_obs.ls);
  Console::printfln("R = %.2f Ω  L = %.4f H", g_obs.rs, g_obs.ls);
  g_obs.reset();
}
static void cmdLam(int argc, char** argv) {
  g_obs.lambda = Console::argf(argv, argc, 1, g_obs.lambda);
  Console::printfln("λm = %.5f Wb", g_obs.lambda);
}

static void cmdFlux(int, char**) {
  Console::printfln("|λ| = %.5f Wb（≈λm，可与 E4.1 的开路法结果对比）", g_obs.fluxMag());
  Console::printfln("|e| = %.3f V   ω̂e = %.1f rad/s", g_obs.bemfMag(), g_obs.omega());
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E4.2 磁链观测器：纯积分 vs 可编程 LPF ===\r\n");
  Console::print("★ 控制仍用编码器，观测器并行运行、不参与控制\r\n");
  Console::print("流程：align → start → spd 20 → mode pure → report ；再 mode lpf → k 0.3 → report\r\n");

  g_con.add("mode",  cmdMode,  "mode pure|lpf");
  g_con.add("k",     cmdK,     "k <v>  ωc = k·|ωe|");
  g_con.add("wc0",   cmdWc0,   "wc0 <rad/s> 最低截止");
  g_con.add("comp",  cmdComp,  "comp 0|1 常数相位补偿");
  g_con.add("subli", cmdSubLi, "subli 0|1 取角度前减 L·i");
  g_con.add("rl",    cmdRL,    "rl <R> <L>");
  g_con.add("lam",   cmdLam,   "lam <Wb>");
  g_con.add("flux",  cmdFlux,  "打印磁链幅值/反EMF");
  obsAddTelemetryNames();
}

void loop() { obsLoop(); }
