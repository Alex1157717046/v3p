/**
 * E4.7 全速域无感 + 误差评估（阶段 4）★ 本方案的压轴实验
 *
 * 目标：完整实现"低速 I/F 强拖 → 高速观测器"的无感运行，并全程用编码器记录角度误差。
 * 验收：画出"角度误差 vs 转速"曲线，清楚看到观测器的可用转速下限。
 *
 * 切换判据（三项一起满足、并持续 N 个周期才切）：
 *   ① 反 EMF 幅值 |ê| > 阈值（越小越早能切，但噪声也越大）
 *   ② 观测器角与 I/F 给定角的偏差 < 30°
 *   ③ 持续 N 个控制周期（防抖）
 *
 * 串口命令（通用命令见 obs_common.h）：
 *   run             跑完整序列（I/F → 自动切换 → 无感）
 *   mode if|obs|auto   手动指定角度来源
 *   th <V>          切换用的反 EMF 阈值
 *   n <cycles>      连续满足多少个周期才切换
 *   swrep           打印切换过程统计
 *   report          打印观测器误差（真值=编码器）
 *   sweepspd        无感状态下自动扫转速出误差曲线
 */

#include <Arduino.h>
#include "obs_common.h"

enum class Sst { Idle, PrePos, Ramp, WaitObs, Sensorless };
static Sst  g_st = Sst::Idle;
static int  g_mode = 2;             // 0=if, 1=obs, 2=auto
static float g_th  = 0.8f;          // 反 EMF 阈值 V
static int   g_needN = 200;         // 连续满足周期数

static float g_thetaRef = 0.0f;
static float g_fNow = 1.0f;
static float g_I = 0.5f;
static float g_alpha = 10.0f;
static int   g_goodN = 0;
static bool  g_switched = false;
static float g_swBemf = 0, g_swF = 0, g_swErr = 0;
static uint32_t g_swMs = 0;

static void cmdRun(int, char**) {
  g_thetaRef = 0.0f; g_fNow = 1.0f; g_goodN = 0; g_switched = false;
  g_m.useExternalAngle = true;
  g_m.externalAngle = 0.0f;
  g_m.enable(true);
  g_running = true;
  g_obs.reset();
  obsClearErr();
  g_st = (g_mode == 1) ? Sst::Sensorless : Sst::PrePos;
  Console::printfln("开始：mode=%s th=%.2f V n=%d", g_mode == 0 ? "if" : g_mode == 1 ? "obs" : "auto",
                    g_th, g_needN);
}

static void cmdMode(int argc, char** argv) {
  if (argc >= 2) {
    if      (!strcmp(argv[1], "if"))   g_mode = 0;
    else if (!strcmp(argv[1], "obs"))  g_mode = 1;
    else if (!strcmp(argv[1], "auto")) g_mode = 2;
  }
  Console::printfln("mode = %s", g_mode == 0 ? "if(强拖)" : g_mode == 1 ? "obs(直接观测器)" : "auto(I/F→观测器)");
}
static void cmdTh(int argc, char** argv) { g_th = Console::argf(argv, argc, 1, g_th);
  Console::printfln("反 EMF 切换阈值 = %.2f V", g_th); }
static void cmdN(int argc, char** argv) { g_needN = (int)Console::argi(argv, argc, 1, g_needN);
  Console::printfln("持续周期数 = %d", g_needN); }
static void cmdIfStop(int, char**) {
  g_st = Sst::Idle; g_m.enable(false); g_m.useExternalAngle = false; g_running = false;
  Console::print("stopped\r\n");
}
static void cmdSwRep(int, char**) {
  Console::printfln("切换状态：%s", g_switched ? "已切换" : "未切换");
  if (g_switched) {
    Console::printfln("切换点：|ê|=%.3f V（阈值 %.2f） 电频率 %.2f Hz  切换瞬间角差 %.2f°  耗时 %u ms",
                      g_swBemf, g_th, g_swF, g_swErr * 180.0f / PI_F, (unsigned)g_swMs);
  }
  Console::printfln("当前：状态=%d  角度源=%s  |ê|=%.3f V", (int)g_st,
                    g_st == Sst::Sensorless ? "观测器" : "I/F 给定", g_obs.bemfMag());
}

// ------------------------------------------------------------------ 单步服务
static void sstServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  switch (g_st) {
    case Sst::PrePos: {
      static uint32_t t0 = 0;
      if (t0 == 0) t0 = now;
      g_thetaRef = 0.0f;
      if (now - t0 > 1000000u) { t0 = 0; g_st = Sst::Ramp; Console::print("→ 频率斜坡\r\n"); }
      break;
    }
    case Sst::Ramp: {
      g_fNow += g_alpha * dt;
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      if (g_obs.bemfMag() > g_th) { g_st = Sst::WaitObs; g_goodN = 0;
                                    Console::print("→ 反 EMF 够了，开始判定切换\r\n"); }
      break;
    }
    case Sst::WaitObs: {
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      const float angDiff = fabsf(wrapPi(g_obs.theta() - g_thetaRef));
      if (g_obs.bemfMag() > g_th && angDiff < (30.0f * PI_F / 180.0f)) g_goodN++;
      else g_goodN = 0;
      if (g_goodN >= g_needN) {
        g_swBemf = g_obs.bemfMag();
        g_swF = g_fNow;
        g_swErr = wrapPi(g_obs.theta() - g_m.thetaE);
        g_swMs = millis();
        g_st = Sst::Sensorless;
        g_switched = true;
        Console::print("→ 切换到观测器，进入无感运行\r\n");
      }
      break;
    }
    case Sst::Sensorless: {
      // 无感：角度全部来自观测器；I/F 的 θ_ref 不再参与控制（但保留用于遥测对比）
      g_thetaRef = wrap2Pi(g_thetaRef + TWO_PI_F * g_fNow * dt);
      break;
    }
    default: break;
  }

  // ---- 控制 ----
  float thetaCmd = g_thetaRef;
  bool useObs = (g_st == Sst::Sensorless) || (g_mode == 1);
  if (useObs) thetaCmd = g_obs.theta();

  if (g_st != Sst::Idle) {
    g_m.useExternalAngle = true;
    g_m.externalAngle = thetaCmd;
    g_m.setCurrentRef(0.0f, g_I);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }

  g_m.update();
  g_obs.update(g_m.vAlphaBeta, g_m.iAlphaBeta, dt, useObs ? g_obs.omega() : g_m.omegaE);

  // 误差统计（真值仍是编码器）
  if (g_st != Sst::Idle && fabsf(g_m.omegaE) > g_obsMinWe) {
    const float e = wrapPi(g_obs.theta() - g_m.thetaE) * 180.0f / PI_F;
    g_errSum += e;
    g_errSum2 += (double)e * e;
    if (fabsf(e) > g_errPeak) g_errPeak = fabsf(e);
    g_errN++;
  }
}

void setup() {
  obsBegin();
  g_obs.type = ObserverType::FluxLpf;
  g_obs.k = 0.3f;
  g_obs.usePll = true;
  g_obs.designPll(TWO_PI_F * 50.0f, 1.0f);

  Console::print("\r\n=== E4.7 全速域无感 + 误差评估（压轴）===\r\n");
  Console::print("流程：align → run → (自动 I/F→观测器) → sweepspd 5 60 5\r\n");

  g_con.add("run",    cmdRun,    "跑完整无感序列");
  g_con.add("mode",   cmdMode,   "mode if|obs|auto");
  g_con.add("th",     cmdTh,     "th <V> 反 EMF 切换阈值");
  g_con.add("n",      cmdN,      "n <周期数>");
  g_con.add("ifstop", cmdIfStop, "停止");
  g_con.add("swrep",  cmdSwRep,  "切换统计");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();
  sstServiceOnce();

  telemetry::set(0, g_m.thetaE * 180.0f / PI_F);
  telemetry::set(1, g_obs.theta() * 180.0f / PI_F);
  telemetry::set(2, wrapPi(g_obs.theta() - g_m.thetaE) * 180.0f / PI_F);
  telemetry::set(3, g_m.omegaE);
  telemetry::set(4, g_obs.omega());
  telemetry::set(5, g_obs.bemfMag());
  telemetry::tick();
}
