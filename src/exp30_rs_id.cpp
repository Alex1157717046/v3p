/**
 * E5.1 Rs 辨识 + 温度补偿（阶段 5）
 *
 * 目标：复现 VESC 的直流注入法 + 温度补偿。
 * 验收：实测温度系数接近 0.0039/°C；补偿后高温下电流环性能不变。
 *
 * 原理：转子堵住 / 对齐后，ωe = 0，电压方程退化为 v = R·i（di/dt=0）
 *       → 注入直流，稳态测 V/I 即得 R。要在【多个电流点】测，看是否线性。
 *
 * ★ 温度补偿（对应模块 09 §9.5 的核心结论）：
 *       R(T) = R0·[1 + α(T − T0)]，铜的 α ≈ 0.00393 /°C
 *       电流环 Ki = ωc·R 与 R 成正比 → 温度升高 Ki 需要跟着变大；
 *       而 Kp = ωc·L 与 R 无关 → 【只补 Ki，不补 Kp】。
 *       本实验让你亲手验证这个结论。
 *
 * 串口命令：
 *   meas <I>       注入 I 安培直流，稳态后测 R
 *   sweep          在 0.1~0.8 A 之间多点测量，看 R 是否随电流变化（饱和/温升）
 *   temp <T0> <T>  设定两点温度，计算补偿系数并打印该怎么改 Ki
 *   r <ohm>        手动设定 R（写进电流环）
 */

#include <Arduino.h>
#include "obs_common.h"

static float g_lastR = 0.0f;
static float g_R0 = 10.0f;      // 冷态电阻
static float g_alphaCu = 0.00393f;

/// 注入指定直流（d 轴），等稳态后返回 vd/id 比值
static float measureR(float I) {
  g_m.voltageMode = false;
  g_m.useExternalAngle = true;
  g_m.externalAngle = 0.0f;        // 固定在 d 轴
  g_m.setCurrentRef(I, 0.0f);
  g_m.enable(true);
  g_running = true;

  // 等 600 ms 让电流稳定（期间控制照跑）
  {
    const uint32_t t0 = millis();
    while (millis() - t0 < 600) { obsServiceOnce(); delay(1); }
  }
  // 采样 400 ms 求平均
  double sv = 0, si = 0;
  int n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 400) {
    obsServiceOnce();
    sv += g_m.vd;
    si += g_m.id;
    n++;
    delay(1);
  }
  if (n < 5 || fabsf((float)(si / n)) < 1e-3f) return -1.0f;
  return (float)(sv / n) / (float)(si / n);
}

static void cmdMeas(int argc, char** argv) {
  const float I = Console::argf(argv, argc, 1, 0.3f);
  const float R = measureR(I);
  if (R < 0) { Console::print("电流太小，测不出（先 align）\r\n"); return; }
  g_lastR = R;
  Console::printfln("I=%.3f A  →  R = %.4f Ω", I, R);
  if (g_m.enc.agc() > 0) {
    // AGC 与磁铁状态顺带打印（判断编码器是否可靠）
    Console::printfln("（参考：当前 vd=%.3f V id=%.3f A，AGC=%u）", g_m.vd, g_m.id, g_m.enc.agc());
  }
}

static void cmdSweep(int, char**) {
  Console::print("---- R 随电流的变化（应基本恒定；明显上升说明发热或饱和）----\r\n");
  Console::print("  I(A)     R(Ω)\r\n");
  for (float I = 0.1f; I <= 0.81f; I += 0.1f) {
    const float R = measureR(I);
    if (R > 0) Console::printfln("%6.2f  %8.4f", I, R);
    else       Console::printfln("%6.2f  —— 测不出", I);
  }
  g_m.enable(false);
  g_running = false;
  g_m.useExternalAngle = false;
  Console::print("→ 若 R 随 I 明显上升，多半是绕组发热（这正好用来做温度实验）\r\n");
}

static void cmdTemp(int argc, char** argv) {
  const float T0 = Console::argf(argv, argc, 1, 25.0f);
  const float T  = Console::argf(argv, argc, 2, 80.0f);
  const float k  = 1.0f + g_alphaCu * (T - T0);
  Console::printfln("温度 %.0f → %.0f °C，补偿系数 = 1 + %.5f×(T−T0) = %.4f", T0, T, g_alphaCu, k);
  Console::printfln("若冷态 R0 = %.3f Ω，热态 R = %.3f Ω", g_R0, g_R0 * k);
  Console::printfln("电流环：Kp = ωc·L 【不变】(%.5f)，Ki = ωc·R 要 ×%.4f (%.2f → %.2f)",
                    g_m.pidD.kp, k, g_m.pidD.ki, g_m.pidD.ki * k);
  Console::print("→ 这就是「温度只影响 Ki、不影响 Kp」的来历：Kp 只含 L，Ki 只含 R\r\n");
}

static void cmdR(int argc, char** argv) {
  const float R = Console::argf(argv, argc, 1, 10.0f);
  g_obs.rs = R;
  g_m.cfg.rs = R;
  g_m.designCurrentLoop(TWO_PI_F * 300.0f);
  Console::printfln("R = %.3f Ω（已套用到电流环：Kp=%.5f Ki=%.1f）", R, g_m.pidD.kp, g_m.pidD.ki);
}

static void cmdR0(int argc, char** argv) {
  g_R0 = Console::argf(argv, argc, 1, g_R0);
  Console::printfln("冷态 R0 = %.3f Ω", g_R0);
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E5.1 Rs 辨识 + 温度补偿 ===\r\n");
  Console::print("★ 请先锁住转子；流程：align → meas 0.3 → sweep → temp 25 80\r\n");
  Console::print("   温度实验：先用吹风机把电机加热，再测一次 R，对比两次数值\r\n");

  g_con.add("meas",  cmdMeas,  "meas <I> 注入直流测 R");
  g_con.add("sweep", cmdSweep, "多点扫描 R");
  g_con.add("temp",  cmdTemp,  "temp <T0> <T> 计算温度补偿");
  g_con.add("r",     cmdR,     "r <ohm> 手动设定 R");
  g_con.add("r0",    cmdR0,    "r0 <ohm> 冷态电阻");
  obsAddTelemetryNames();
}

void loop() { obsLoop(); }
