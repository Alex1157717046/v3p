/**
 * E4.6 平滑切换（阶段 4）
 *
 * 目标：复现"角度不连续 → 转矩冲击"，并解决。
 * 验收：切换瞬间电流尖峰降低 > 50%；ω 无阶跃。
 *
 * 场景：I/F 强拖到一定转速 → 把控制角度从 θ_ref 切到观测器 θ̂。
 *   硬切换：两角不相等时会有一个突变 → 电流冲击
 *   加权融合：θ = (1−w)·θ_ref + w·θ̂，w 在 T_blend 内 0→1（需要 wrap_pi 处理跨界）
 *   进阶（VESC 做法）：切换前先给观测器预置 θ_ref ± 45°，让它自己收敛，做到"无需融合"
 *
 * 串口命令：
 *   run             跑完整序列（I/F → 切换），自动测电流尖峰
 *   hard 0|1        0=加权融合，1=硬切换
 *   blend <ms>      融合时间
 *   preset 0|1      切换前预置观测器角度
 *   report
 */

#include <Arduino.h>
#include "obs_common.h"

enum class St { Idle, PrePos, Ramp, Hold, Blend, Done };
static St g_st = St::Idle;

static bool  g_hard = false;
static float g_blendMs = 300.0f;
static bool  g_preset = false;
static float g_thetaRef = 0.0f;
static float g_fNow = 1.0f;
static float g_I = 0.4f;
static float g_alpha = 8.0f;
static float g_fSwitch = 15.0f;     // 到该电频率后开始切换

static float g_w = 0.0f;
static uint32_t g_tBlendStart = 0;
static float g_iqBefore = 0.0f;
static float g_spikePeak = 0.0f;
static float g_dSpikePeak = 0.0f;

static void cmdHard(int argc, char** argv) {
  g_hard = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("切换方式 = %s", g_hard ? "硬切换" : "加权融合");
}
static void cmdBlend(int argc, char** argv) {
  g_blendMs = Console::argf(argv, argc, 1, g_blendMs);
  Console::printfln("融合时间 = %.0f ms", g_blendMs);
}
static void cmdPreset(int argc, char** argv) {
  g_preset = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("切换前预置观测器：%s", g_preset ? "ON（VESC 式）" : "OFF");
}

static void cmdRun(int, char**) {
  g_thetaRef = 0.0f;
  g_fNow = 1.0f;
  g_w = 0.0f;
  g_spikePeak = 0.0f;
  g_dSpikePeak = 0.0f;
  g_m.useExternalAngle = true;
  g_m.externalAngle = 0.0f;
  g_m.enable(true);
  g_running = true;
  g_obs.reset();
  g_st = St::PrePos;
  Console::printfln("开始序列：预定位 → 斜坡 → 到 %.0f Hz 切换（%s，融合 %.0f ms）", g_fSwitch,
                    g_hard ? "硬切换" : "融合", g_blendMs);
}

static void cmdSwitchReport(int, char**) {
  Console::printfln("切换方式=%s  融合=%.0f ms  预置=%d", g_hard ? "硬" : "融合", g_blendMs,
                    g_preset ? 1 : 0);
  Console::printfln("iq 尖峰 = %.4f A（切换前 iq=%.4f A，变化 %.1f%%）", g_spikePeak, g_iqBefore,
                    100.0f * (g_spikePeak - fabsf(g_iqBefore)) / (fabsf(g_iqBefore) + 1e-6f));
  Console::printfln("角度误差尖峰 = %.2f°", g_dSpikePeak * 180.0f / PI_F);
  Console::print("→ 融合/预置应让尖峰明显小于硬切换；两种方式各跑一次对比\r\n");
}

void setup() {
  obsBegin();
  g_obs.type = ObserverType::FluxLpf;
  g_obs.k = 0.3f;
  g_obs.usePll = true;
  g_obs.designPll(TWO_PI_F * 50.0f, 1.0f);

  Console::print("\r\n=== E4.6 平滑切换（I/F → 观测器）===\r\n");
  Console::print("流程：align → start(编码器) 让观测器收敛 → run ；再 hard 1 → run 对比\r\n");
  Console::print("注：观测器需要一定的反 EMF 才能工作，所以先用 I/F 把转速拖起来\r\n");

  g_con.add("run",    cmdRun,    "跑完整序列并测尖峰");
  g_con.add("hard",   cmdHard,   "hard 0|1");
  g_con.add("blend",  cmdBlend,  "blend <ms>");
  g_con.add("preset", cmdPreset, "preset 0|1");
  g_con.add("swrep",  cmdSwitchReport, "打印尖峰统计");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();

  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  const float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;

  switch (g_st) {
    case St::PrePos: {
      static uint32_t t0 = 0;
      if (t0 == 0) t0 = now;
      g_thetaRef = 0.0f;
      if (now - t0 > 1000000u) { t0 = 0; g_st = St::Ramp; }
      break;
    }
    case St::Ramp: {
      g_fNow += g_alpha * dt;
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      if (g_fNow >= g_fSwitch) {
        // 让观测器先收敛一会儿（用 θ_ref 作为参考速度）
        if (g_preset) g_obs.presetAngle(g_thetaRef);
        g_st = St::Hold;
        static uint32_t t1 = 0;
        t1 = now;
        (void)t1;
        Console::print("到切换转速，进入过渡\r\n");
      }
      break;
    }
    case St::Hold: {
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      static uint32_t tHold = 0;
      if (tHold == 0) { tHold = now; g_w = 0.0f; g_tBlendStart = now; g_iqBefore = g_m.iq;
                        Console::print("→ 开始切换\r\n"); }
      if (g_hard) {
        g_w = 1.0f;
        g_st = St::Blend;
      } else if (now - tHold > 400000u) {   // 先让观测器跟 0.4 s
        tHold = 0;
        g_st = St::Blend;
        g_tBlendStart = now;
      }
      break;
    }
    case St::Blend: {
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      if (!g_hard) {
        g_w = fminf(1.0f, (float)(now - g_tBlendStart) / (g_blendMs * 1000.0f));
      }
      if (g_w >= 1.0f) { g_st = St::Done; Console::print("→ 切换完成，纯观测器运行\r\n"); }
      break;
    }
    default: break;
  }

  // ---- 控制角度：θ = (1−w)·θ_ref + w·θ̂（用 wrap_pi 处理跨界）----
  float thetaCmd = g_thetaRef;
  if (g_st == St::Blend || g_st == St::Done) {
    const float d = wrapPi(g_obs.theta() - g_thetaRef);
    thetaCmd = wrap2Pi(g_thetaRef + g_w * d);
  }

  if (g_st != St::Idle) {
    g_m.useExternalAngle = true;
    g_m.externalAngle = thetaCmd;
    g_m.setCurrentRef(0.0f, g_I);
    // 观测器并行运行（它已经在 obsLoop 里更新；这里我们自己在开关后仍要更新）
    g_obs.update(g_m.vAlphaBeta, g_m.iAlphaBeta, dt, g_m.omegaE);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }

  g_m.update();

  // ---- 尖峰统计（切换后 200 ms 窗口）----
  if (g_st == St::Blend || g_st == St::Done) {
    const float dIq = fabsf(g_m.iq);
    if (dIq > g_spikePeak) g_spikePeak = dIq;
    const float dAng = fabsf(wrapPi(g_thetaRef - g_m.thetaE));
    if (dAng > g_dSpikePeak) g_dSpikePeak = dAng;
  }

  telemetry::set(0, g_thetaRef * 180.0f / PI_F);
  telemetry::set(1, thetaCmd * 180.0f / PI_F);
  telemetry::set(2, g_m.thetaE * 180.0f / PI_F);
  telemetry::set(3, g_m.iq);
  telemetry::set(4, g_m.id);
  telemetry::set(5, g_w);
  telemetry::tick();
}
