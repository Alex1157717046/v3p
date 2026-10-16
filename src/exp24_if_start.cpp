/**
 * E4.5 I/F 强拖启动 ★ 用编码器测负载角 δ（阶段 4）
 *
 * 目标：实现三段式启动（预定位 → 频率斜坡 → 恒速），并用编码器【直接测出负载角 δ】。
 * 验收：δ 随 I 增大而减小、随斜坡率 α 增大而增大，定量吻合
 *           sinδ = (T_L + J·α) / (Kt·I)
 *
 * ★ 价值：在【有真值】的条件下验证教科书公式 —— 这是极难得的机会。
 *
 * 做法：控制用【外部给定角度】θ_ref（I/F 强拖），
 *       同时读编码器真实电角度 θ_true，
 *       δ = wrapPi(θ_ref − θ_true)   ← 直接得到，不需要任何估计
 *
 * 串口命令：
 *   if <I> <f0> <alpha> <f1>   启动 I/F 序列
 *        I     强拖电流 (A)
 *        f0    起始电频率 (Hz)
 *        alpha 斜坡率 (Hz/s)
 *        f1    目标电频率 (Hz)
 *   stop
 *   delta       打印当前 δ
 *   compare     自动跑几组 (I, α) 并打印 δ 表
 */

#include <Arduino.h>
#include "obs_common.h"

enum class IfState { Idle, PrePos, Ramp, Hold };
static IfState g_st = IfState::Idle;

static float g_I = 0.4f;          // 强拖电流 A
static float g_f0 = 1.0f;         // 起始电频率 Hz
static float g_alpha = 5.0f;      // Hz/s
static float g_f1 = 10.0f;        // 目标电频率 Hz
static float g_thetaRef = 0.0f;   // 给定电角度
static float g_fNow = 0.0f;       // 当前电频率
static float g_delta = 0.0f;      // 负载角（rad）
static float g_deltaAcc = 0; static int g_deltaN = 0;

static void ifServiceOnce();   // 前置声明（命令里要用）

static void startIf(float I, float f0, float alpha, float f1) {
  g_I = I; g_f0 = f0; g_alpha = alpha; g_f1 = f1;
  g_thetaRef = 0.0f; g_fNow = f0;
  g_deltaAcc = 0; g_deltaN = 0;
  g_m.useExternalAngle = true;
  g_m.externalAngle = 0.0f;
  g_m.enable(true);
  g_st = IfState::PrePos;
  Console::printfln("I/F: I=%.2f A  f0=%.2f Hz  α=%.1f Hz/s  f1=%.2f Hz", I, f0, alpha, f1);
}

static void cmdIf(int argc, char** argv) {
  startIf(Console::argf(argv, argc, 1, g_I), Console::argf(argv, argc, 2, g_f0),
          Console::argf(argv, argc, 3, g_alpha), Console::argf(argv, argc, 4, g_f1));
}

static void cmdIfStop(int, char**) {
  g_st = IfState::Idle;
  g_m.enable(false);
  g_m.useExternalAngle = false;
  Console::print("stopped\r\n");
}

static void cmdDelta(int, char**) {
  Console::printfln("δ = %.3f°  (θ_ref=%.1f°, θ_true=%.1f°, f=%.2f Hz, I=%.2f A)",
                    g_delta * 180.0f / PI_F, g_thetaRef * 180.0f / PI_F,
                    g_m.thetaE * 180.0f / PI_F, g_fNow, g_I);
}

/// 自动跑几组 (I, α)，打印 δ —— 用来验证 sinδ = (TL + Jα)/(Kt·I)
static void cmdCompare(int, char**) {
  Console::print("---- I/F 负载角对比（目标电频率 8 Hz）----\r\n");
  Console::print("  I(A)   α(Hz/s)   δ(°)    说明\r\n");
  const float Is[] = {0.2f, 0.4f, 0.8f};
  const float as[] = {2.0f, 5.0f, 15.0f};
  for (float a : as) {
    for (float I : Is) {
      startIf(I, 0.5f, a, 8.0f);
      {                                  // ★ 等待期间必须继续跑控制
        const uint32_t tw = millis();
        while (millis() - tw < 4000) { ifServiceOnce(); delay(1); }
      }
      const float d = g_delta * 180.0f / PI_F;
      Console::printfln("%6.2f  %8.1f  %8.2f    %s", I, a, d,
                        (g_deltaN > 0) ? "已测" : "未到恒速");
      g_m.enable(false);
      { const uint32_t tw = millis(); while (millis() - tw < 500) { ifServiceOnce(); delay(1); } }
    }
  }
  Console::print("→ 期望：I 越大 δ 越小；α 越大 δ 越大\r\n");
  g_st = IfState::Idle;
  g_m.useExternalAngle = false;
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E4.5 I/F 强拖启动 + 负载角 δ 实测 ===\r\n");
  Console::print("流程：align → if 0.4 1 5 10 → delta ；或直接 compare\r\n");

  g_con.add("if",      cmdIf,      "if <I> <f0> <alpha> <f1>");
  g_con.add("ifstop",  cmdIfStop,  "停止 I/F");
  g_con.add("delta",   cmdDelta,   "打印负载角");
  g_con.add("compare", cmdCompare, "自动对比多组 I 与 α");
  obsAddTelemetryNames();
}

// 单步服务：状态机 + 控制（loop() 与阻塞式命令都调用它）
static void ifServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  switch (g_st) {
    case IfState::PrePos: {
      // 预定位：θ_ref 固定为 0，保持 1.2 s
      static uint32_t tEnter = 0;
      if (tEnter == 0) tEnter = now;
      g_thetaRef = 0.0f;
      if (now - tEnter > 1200000u) { tEnter = 0; g_st = IfState::Ramp; Console::print("→ 斜坡\r\n"); }
      break;
    }
    case IfState::Ramp: {
      g_fNow += g_alpha * dt;
      if (g_fNow >= g_f1) { g_fNow = g_f1; g_st = IfState::Hold; g_deltaAcc = 0; g_deltaN = 0;
                            Console::print("→ 恒速（开始统计 δ）\r\n"); }
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      break;
    }
    case IfState::Hold: {
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      break;
    }
    default: break;
  }

  // ---- 控制：外部角度 + 纯 iq 电流 ----
  if (g_st != IfState::Idle) {
    g_m.useExternalAngle = true;
    g_m.externalAngle = g_thetaRef;
    g_m.setCurrentRef(0.0f, g_I);
    g_running = true;
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }

  g_m.update();

  // ---- δ = θ_ref − θ_true ----
  g_delta = wrapPi(g_thetaRef - g_m.thetaE);
  if (g_st == IfState::Hold) { g_deltaAcc += g_delta; g_deltaN++; }
}

void loop() {
  g_con.poll();
  ifServiceOnce();

  telemetry::set(0, g_thetaRef * 180.0f / PI_F);
  telemetry::set(1, g_m.thetaE * 180.0f / PI_F);
  telemetry::set(2, g_delta * 180.0f / PI_F);
  telemetry::set(3, g_fNow);
  telemetry::set(4, g_m.iq);
  telemetry::set(5, (float)g_st);
  telemetry::tick();
}
