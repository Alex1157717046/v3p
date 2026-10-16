/**
 * E6.2 弱磁实现（阶段 6）
 *
 * 目标：实现弱磁，突破基速。
 * 验收：转速提升 ≥ 20%；id 随转速变负。
 *
 * 原理：电压极限 vd²+vq² ≤ Vmax²。转速升高时 ωe·λm 吃掉全部电压预算，
 *       此时给一个负的 id（去磁电流）可以把 vq 的需求拉回来，从而继续提速。
 *       代价是 id 会产生额外铜耗，且过大的负 id 有退磁风险。
 *
 * ⚠ 12.6 E6.2 的提醒：2208 云台电机 λm 大、转速低，12 V 下可能进不了弱磁区。
 *   本工程给了两个"制造场景"的手段：
 *     ① vlim <V>   把电压限幅人为调低（等价于降压供电）→ 立刻就能进入弱磁
 *     ② 用高速/低阻电机
 *
 * 弱磁控制器：vlim 检测型
 *   |v| > Vmax·0.97 → idRef 往负方向走
 *   |v| < Vmax·0.85 → idRef 往 0 回
 *
 * 串口命令：
 *   fw 0|1          弱磁开关
 *   vlim <V>        电压限幅（默认 Vdc/√3·0.95，调低可强制进入弱磁）
 *   idmax <A>       允许的最大去磁电流（绝对值）
 *   spd <rad/s>     目标转速
 *   max             自动升速直到顶不住，记录最高转速（fw 开/关各跑一次对比）
 */

#include <Arduino.h>
#include "obs_common.h"

static bool  g_fw = false;
static float g_idFw = 0.0f;
static float g_idMax = 0.35f;
static float g_vlim = 0.0f;

static void fwServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  if (g_running) {
    const float iq = g_pidSpd.update(g_spdRef - g_m.enc.velocity(), dt);

    // ---- 弱磁：按电压余量调整 id ----
    if (g_fw) {
      const float vmag = sqrtf(g_m.vd * g_m.vd + g_m.vq * g_m.vq);
      const float hi = g_vlim * 0.97f, lo = g_vlim * 0.85f;
      if (vmag > hi)      g_idFw -= 2.0f * dt;      // 电压吃紧 → 加去磁
      else if (vmag < lo) g_idFw += 1.0f * dt;      // 电压宽松 → 退回
      g_idFw = fmaxf(-g_idMax, fminf(0.0f, g_idFw));
    } else {
      g_idFw = 0.0f;
    }
    g_m.setCurrentRef(g_idFw, iq);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }
  g_m.update();
}

static void fwWait(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) { fwServiceOnce(); delay(1); }
}

static void cmdFw(int argc, char** argv) {
  g_fw = Console::argi(argv, argc, 1, 1) != 0;
  if (!g_fw) g_idFw = 0.0f;
  Console::printfln("弱磁 %s（idmax=%.2f A）", g_fw ? "ON" : "OFF", g_idMax);
}

static void cmdVlim(int argc, char** argv) {
  g_vlim = Console::argf(argv, argc, 1, g_vlim);
  g_m.setVoltageLimit(g_vlim);
  Console::printfln("电压限幅 = %.3f V（默认 %.3f V）", g_vlim, VDC_DEFAULT * ONE_BY_SQRT3_F * 0.95f);
  Console::print("→ 调小它就等于降压供电，能立刻把电机推进弱磁区（12.6 E6.2 的技巧）\r\n");
}

static void cmdIdMax(int argc, char** argv) {
  g_idMax = Console::argf(argv, argc, 1, g_idMax);
  Console::printfln("idmax = %.3f A", g_idMax);
}

/// 自动升速，找到当前配置下的最高转速
static void cmdMax(int, char**) {
  if (!g_running) { Console::print("先 start\r\n"); return; }
  Console::printfln("---- 升速测试（弱磁 %s，电压限幅 %.2f V）----", g_fw ? "ON" : "OFF", g_vlim);
  Console::print(" ωref(rad/s)  rpm    实测ωm      id(A)    iq(A)    |v|(V)   |i|(A)\r\n");
  float wMax = 0.0f;
  for (float w = 10.0f; w <= 200.0f; w += 10.0f) {
    g_spdRef = w;
    fwWait(1800);
    const float wm = g_m.enc.velocity();
    const float imag = sqrtf(g_m.id * g_m.id + g_m.iq * g_m.iq);
    const float vmag = sqrtf(g_m.vd * g_m.vd + g_m.vq * g_m.vq);
    Console::printfln("%11.1f  %5.0f  %8.2f  %8.3f  %7.3f  %7.2f  %7.3f", w, w * 60.0f / TWO_PI_F, wm,
                      g_m.id, g_m.iq, vmag, imag);
    if (wm > wMax) wMax = wm;
    // 顶不住就退出（实测转速显著低于给定，且电流已到限幅）
    if (wm < w * 0.75f && imag > g_m.cfg.currentLimit * 0.95f) {
      Console::print("→ 已顶不住（电流到限、转速跟不上），停止升速\r\n");
      break;
    }
  }
  Console::printfln("★ 最高实测转速 = %.2f rad/s (%.0f rpm)", wMax, wMax * 60.0f / TWO_PI_F);
  Console::print("→ fw 0 与 fw 1 各跑一次 max，比较最高转速；弱磁应提升 ≥ 20%\r\n");
  g_spdRef = 0.0f;
}

void setup() {
  obsBegin();
  g_vlim = VDC_DEFAULT * ONE_BY_SQRT3_F * 0.95f;
  g_m.setVoltageLimit(g_vlim);

  Console::print("\r\n=== E6.2 弱磁实现 ===\r\n");
  Console::print("★ 2208 在 12 V 下可能进不了弱磁区 → 先用 vlim 3 人为制造电压受限场景\r\n");
  Console::print("流程：align → start → vlim 3 → fw 0 → max ；再 fw 1 → max 对比\r\n");

  g_con.add("fw",    cmdFw,    "fw 0|1");
  g_con.add("vlim",  cmdVlim,  "vlim <V> 电压限幅");
  g_con.add("idmax", cmdIdMax, "idmax <A> 去磁电流上限");
  g_con.add("max",   cmdMax,   "自动升速找最高转速");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();
  fwServiceOnce();

  telemetry::set(0, g_m.enc.velocity());
  telemetry::set(1, g_spdRef);
  telemetry::set(2, g_m.id);
  telemetry::set(3, g_m.iq);
  telemetry::set(4, sqrtf(g_m.vd * g_m.vd + g_m.vq * g_m.vq));
  telemetry::set(5, g_fw ? 1.0f : 0.0f);
  telemetry::tick();
}
