/**
 * E5.5 效率测量与优化（阶段 5）
 *
 * 目标：测量并优化效率。
 * 验收：画出效率-转速、效率-转矩曲线；优化后提升 ≥ 3%。
 *
 * ⚠ 这块板的限制（必须说清楚）：
 *   · V3P 有 POWER_ADC（母线电压），但【没有母线电流传感器】→ 无法直接测输入功率
 *   · STM32 转接板更是连 POWER_ADC 都没引出来
 *   → 所以本实验用 12.6 E5.5 里给的【代理指标】：
 *        "相同转速下电流最小" = 效率最高
 *     对同一个转速，扫 id（d 轴电流），看总电流幅值 |i| 与铜耗 1.5·Rs·|i|² 何时最小。
 *
 *   对表贴式电机（SPM，凸极率≈1）最小值应出现在 id≈0 —— 这本身就验证了
 *   "id=0 控制"对云台电机是最优的。
 *
 * 串口命令：
 *   spd <rad/s>     测试转速
 *   sweep           扫 id（-0.4 → +0.4）并打印 |i| 与铜耗
 *   id <A>          手动设定 id
 *   r <ohm>         设定 Rs（算铜耗用）
 */

#include <Arduino.h>
#include "obs_common.h"

static float g_idRef = 0.0f;
static float g_rs = 10.0f;
static float g_iqNow = 0.0f;

static void effServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  if (g_running) {
    g_iqNow = g_pidSpd.update(g_spdRef - g_m.enc.velocity(), dt);
    g_m.setCurrentRef(g_idRef, g_iqNow);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }
  g_m.update();
}

static void effWait(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) { effServiceOnce(); delay(1); }
}

static void cmdEffSpd(int argc, char** argv) {
  g_spdRef = Console::argf(argv, argc, 1, 20.0f);
  Console::printfln("测试转速 = %.1f rad/s (%.0f rpm)", g_spdRef, g_spdRef * 60.0f / TWO_PI_F);
}

static void cmdId(int argc, char** argv) {
  g_idRef = Console::argf(argv, argc, 1, 0.0f);
  Console::printfln("id_ref = %.3f A", g_idRef);
}

static void cmdR(int argc, char** argv) {
  g_rs = Console::argf(argv, argc, 1, g_rs);
  Console::printfln("Rs = %.3f Ω", g_rs);
}

/// 扫 id，找"同转速下电流最小"
static void cmdSweep(int, char**) {
  if (!g_running) cmdStart(0, nullptr);
  Console::printfln("---- 效率扫描 @ ωm=%.1f rad/s ----", g_spdRef);
  Console::printfln("  id(A)    iq(A)     |i|(A)    铜耗 1.5Rs|i|²(W)");
  float best = 1e9f, bestId = 0, bestI = 0;
  for (float id = -0.4f; id <= 0.401f; id += 0.1f) {
    g_idRef = id;
    effWait(900);                       // 稳定
    // 采样平均
    double si = 0, sq = 0;
    int n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 400) { effServiceOnce(); si += g_m.id; sq += g_m.iq; n++; delay(1); }
    const float idm = (float)(si / n), iqm = (float)(sq / n);
    const float imag = sqrtf(idm * idm + iqm * iqm);
    const float pcu = 1.5f * g_rs * imag * imag;
    Console::printfln("%6.2f   %7.3f   %8.4f   %12.4f", idm, iqm, imag, pcu);
    if (imag < best) { best = imag; bestId = idm; bestI = imag; }
  }
  Console::printfln("→ 最小电流点：id = %.3f A，|i| = %.4f A", bestId, bestI);
  Console::print("→ 若最小点在 id≈0，说明这台表贴式电机的 MTPA 就是 id=0（无需弱磁时的最优）\r\n");
  g_idRef = 0.0f;
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E5.5 效率测量与优化（代理指标法）===\r\n");
  Console::print("★ 本板没有母线电流传感器，无法直接测输入功率，用「同转速电流最小」作代理\r\n");
  Console::print("流程：align → start → espd 20 → sweep\r\n");

  g_con.add("espd",  cmdEffSpd, "espd <rad/s> 测试转速");
  g_con.add("id",    cmdId,    "id <A>");
  g_con.add("ers",   cmdR,     "ers <ohm>");
  g_con.add("sweep", cmdSweep, "扫 id 找最小电流点");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();
  // 复用 obs_common 的 start/stop 会走 obsServiceOnce，这里统一用自己的服务
  effServiceOnce();

  telemetry::set(0, g_m.id);
  telemetry::set(1, g_m.iq);
  telemetry::set(2, sqrtf(g_m.id * g_m.id + g_m.iq * g_m.iq));
  telemetry::set(3, g_m.enc.velocity());
  telemetry::set(4, g_spdRef);
  telemetry::set(5, 1.5f * g_rs * (g_m.id * g_m.id + g_m.iq * g_m.iq));
  telemetry::tick();
}
