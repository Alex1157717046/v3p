/**
 * E6.1 电压极限椭圆实验（阶段 6）
 *
 * 目标：实测电机的"电压极限"轨迹。
 * 验收：实测轨迹与理论椭圆吻合。
 * ★ 价值：把一张理论图变成【你自己测出来的图】。
 *
 * 理论（稳态、id=0）：
 *   vd = −ωe·L·iq
 *   vq =  R·iq + ωe·λm
 *   vd² + vq² ≤ Vmax²      Vmax = Vdc/√3（SVPWM 线性区极限）
 * 扫 iq → 每个转速下"刚好饱和"的工作点就是电压极限点。
 *
 * 做法：速度环把转速顶住，逐渐加大 iq 给定直到 vLimitScale < 1（= 电压已饱和），
 *       记录该点的 iq、|v|、ωe，写进表。
 *
 * 串口命令：
 *   vdc <V>         母线电压（用于算 Vmax）
 *   spd <rad/s>     目标转速
 *   probe           在当前转速逐步加 iq 找饱和点
 *   map <n>         自动在 n 个转速下各找一次饱和点，打印整张表
 *   start / stop
 */

#include <Arduino.h>
#include "obs_common.h"

static float g_vdc = VDC_DEFAULT;
static float g_iqProbe = 0.0f;
static float g_vmax = 0.0f;

static void vServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  if (g_running) {
    // 转速环给一个基础 iq，再叠加探测用的 iq
    const float iqBase = g_pidSpd.update(g_spdRef - g_m.enc.velocity(), dt);
    g_iqProbe = iqBase;
    g_m.setCurrentRef(0.0f, g_iqProbe);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }
  g_m.update();
}

static void vWait(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) { vServiceOnce(); delay(1); }
}

static void cmdVdc(int argc, char** argv) {
  g_vdc = Console::argf(argv, argc, 1, g_vdc);
  g_m.cfg.vdc = g_vdc;
  g_vmax = g_vdc * ONE_BY_SQRT3_F;
  Console::printfln("Vdc = %.2f V  →  Vmax(SVPWM 线性区) = Vdc/√3 = %.3f V", g_vdc, g_vmax);
  Console::printfln("SPWM 的极限只有 Vdc/2 = %.3f V（对比用）", g_vdc * 0.5f);
}

static void cmdVSpd(int argc, char** argv) {
  g_spdRef = Console::argf(argv, argc, 1, 20.0f);
  Console::printfln("目标转速 = %.1f rad/s", g_spdRef);
}

/// 在当前转速下逐步增大 iq 直到饱和
static void probeOnce(float spd, float* outIq, float* outV, float* outWe) {
  g_spdRef = spd;
  g_pidSpd.reset();
  vWait(1500);                      // 稳住转速
  for (int k = 0; k < 40; k++) {
    vWait(60);
    if (g_m.vLimitScale < 0.995f) break;      // 已饱和
    // 通过临时抬高电流限幅来加大 iq
    g_m.cfg.currentLimit += 0.05f;
  }
  *outIq = sqrtf(g_m.id * g_m.id + g_m.iq * g_m.iq);
  *outV  = sqrtf(g_m.vd * g_m.vd + g_m.vq * g_m.vq);
  *outWe = fabsf(g_m.omegaE);
  g_m.cfg.currentLimit = 1.0f;
}

static void cmdProbe(int, char**) {
  float iq, v, we;
  probeOnce(g_spdRef, &iq, &v, &we);
  Console::printfln("转速 %.1f rad/s (%.0f rpm): 饱和点 |i|=%.4f A  |v|=%.3f V  ωe=%.1f rad/s",
                    g_spdRef, g_spdRef * 60.0f / TWO_PI_F, iq, v, we);
  Console::printfln("理论 vq = R·iq + ωe·λm = %.2f×%.3f + %.1f×%.5f = %.3f V",
                    g_obs.rs, g_m.iq, we, g_obs.lambda, g_obs.rs * g_m.iq + we * g_obs.lambda);
  Console::printfln("理论 vd = −ωe·L·iq = %.3f V   →  |v|理论 = %.3f V（实测 %.3f）", -we * g_obs.ls * g_m.iq,
                    sqrtf(powf(g_obs.rs * g_m.iq + we * g_obs.lambda, 2) + powf(we * g_obs.ls * g_m.iq, 2)), v);
}

/// 自动在多个转速下找饱和点
static void cmdMap(int argc, char** argv) {
  const int n = (int)Console::argi(argv, argc, 1, 8);
  Console::printfln("---- 电压极限点扫描（Vdc=%.1f V, Vmax=%.2f V）----", g_vdc, g_vmax);
  Console::print(" ωm(rad/s)  rpm    ωe(rad/s)  饱和|i|(A)  |v|(V)   理论|v|(V)\r\n");
  const float from = 5.0f, to = 80.0f;
  for (int k = 0; k < n; k++) {
    const float spd = from + (to - from) * (float)k / (float)(n - 1);
    float iq, v, we;
    probeOnce(spd, &iq, &v, &we);
    const float vth = sqrtf(powf(g_obs.rs * g_m.iq + we * g_obs.lambda, 2) +
                            powf(we * g_obs.ls * g_m.iq, 2));
    Console::printfln("%9.1f  %5.0f  %9.1f  %10.4f  %8.3f  %10.3f", spd, spd * 60.0f / TWO_PI_F, we,
                      iq, v, vth);
  }
  Console::print("→ 把 (id, iq) 或 (ωe, iq) 画到平面上就是你的实测电压极限轨迹\r\n");
  Console::print("→ 与理论椭圆对比：低转速时受电流限幅（圆），高转速时受电压限幅（椭圆收缩）\r\n");
}

void setup() {
  obsBegin();
  g_vmax = g_vdc * ONE_BY_SQRT3_F;
  Console::print("\r\n=== E6.1 电压极限椭圆 ===\r\n");
  Console::print("流程：align → vdc 12 → start → map 8\r\n");
  Console::print("★ 先确保 E4.1 已辨识出 λm、E5.1/E5.2 已辨识 R/L（否则理论值对不上）\r\n");

  g_con.add("vdc",   cmdVdc,    "vdc <V>");
  g_con.add("vspd",  cmdVSpd,   "vspd <rad/s>");
  g_con.add("probe", cmdProbe,  "在当前转速找饱和点");
  g_con.add("map",   cmdMap,    "map <n> 自动扫多个转速");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();
  vServiceOnce();

  telemetry::set(0, g_m.omegaE);
  telemetry::set(1, g_m.iq);
  telemetry::set(2, g_m.id);
  telemetry::set(3, sqrtf(g_m.vd * g_m.vd + g_m.vq * g_m.vq));
  telemetry::set(4, g_vmax);
  telemetry::set(5, g_m.vLimitScale);
  telemetry::tick();
}
