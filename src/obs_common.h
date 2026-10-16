/**
 * @file obs_common.h
 * @brief 阶段 4（观测器）实验的共用评测台
 *
 * 核心方法（对应 12.3）：
 *   · 控制【仍然用 AS5600 编码器】，保证电机不失控
 *   · 观测器【并行运行但不参与控制】
 *   · 用编码器角度当真值，量化观测器误差 → 直接得到"误差 vs 转速"曲线
 *
 * ⚠ 重要实现约束（踩过的坑）：
 *   Arduino 的 loop() 和"阻塞式命令"是同一个线程 —— 命令里 delay() 的时候 loop() 不会执行。
 *   所以任何需要"边等边跑控制"的命令（扫转速、对比几种参数）都**必须**在等待循环里
 *   调用 obsServiceOnce()，否则电机会失控、样本也采不到。
 */
#pragma once

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t OBS_BAUD = 921600;

static FocMotor     g_m;
static Observer     g_obs;
static Console      g_con;
static PiController g_pidSpd;

static bool  g_running = false;
static float g_spdRef  = 0.0f;      // 机械角速度给定 rad/s
static float g_wcsHz   = 5.0f;
static float g_obsMinWe = 30.0f;    // 只在该电角速度以上统计误差（反 EMF 足够大）

// 误差统计
static float g_errSum = 0, g_errSum2 = 0, g_errPeak = 0;
static int   g_errN = 0;

static void obsDesignSpeed(float hz) {
  const float wc = TWO_PI_F * hz;
  g_pidSpd.kp = wc * 2.0e-6f / 0.083f;
  g_pidSpd.ki = g_pidSpd.kp * wc * 0.5f / wc;   // 保持 Ki = Kp·ωc/2 的形式
  g_pidSpd.ki = wc * g_pidSpd.kp * 0.5f;
  g_pidSpd.outMin = -g_m.cfg.currentLimit;
  g_pidSpd.outMax =  g_m.cfg.currentLimit;
  g_pidSpd.intMin = g_pidSpd.outMin;
  g_pidSpd.intMax = g_pidSpd.outMax;
  g_pidSpd.aw = AntiWindup::Clamp;
}

static void obsClearErr() { g_errSum = g_errSum2 = g_errPeak = 0; g_errN = 0; }

// ============================================================================
//  单步服务：推进速度环 + 电流环 + 观测器 + 误差统计
//  loop() 里每轮调用一次；任何"阻塞等待"的循环里也必须调用它
// ============================================================================
static void obsServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  if (g_running) {
    const float iqRef = g_pidSpd.update(g_spdRef - g_m.enc.velocity(), dt);
    g_m.setCurrentRef(0.0f, iqRef);
  } else {
    g_m.setCurrentRef(0.0f, 0.0f);
  }

  g_m.update();
  g_obs.update(g_m.vAlphaBeta, g_m.iAlphaBeta, dt, g_m.omegaE);

  if (g_running && fabsf(g_m.omegaE) > g_obsMinWe) {
    const float e = wrapPi(g_obs.theta() - g_m.thetaE) * 180.0f / PI_F;
    g_errSum += e;
    g_errSum2 += (double)e * e;
    if (fabsf(e) > g_errPeak) g_errPeak = fabsf(e);
    g_errN++;
  }
}

/// 阻塞式等待，但期间仍然跑控制（ms 毫秒）
static void obsWait(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    obsServiceOnce();
    delay(1);
  }
}

// ============================================================================
//  命令
// ============================================================================

static void cmdAlign(int, char**) {
  Console::print("aligning...\r\n");
  g_m.alignSensor(1.0f, 1500);
  g_m.enc.setZero();
  g_obs.reset();
  obsClearErr();
  Console::print("done（编码器已清零，观测器已复位）\r\n");
}

static void cmdStart(int, char**) {
  g_pidSpd.reset();
  g_obs.reset();
  obsClearErr();
  g_m.enable(true);
  g_running = true;
  Console::print("running（控制用编码器，观测器并行运行）\r\n");
}

static void cmdStop(int, char**) { g_running = false; g_m.enable(false); }

static void cmdSpd(int argc, char** argv) {
  g_spdRef = Console::argf(argv, argc, 1, g_spdRef);
  obsClearErr();
  Console::printfln("ω_ref = %.2f rad/s (电角速度 %.1f rad/s)", g_spdRef,
                    g_spdRef * (float)g_m.cfg.polePairs);
}

static void cmdWcs(int argc, char** argv) {
  g_wcsHz = Console::argf(argv, argc, 1, g_wcsHz);
  obsDesignSpeed(g_wcsHz);
  Console::printfln("速度环 ωc=%.1f Hz", g_wcsHz);
}

static void cmdErrClear(int, char**) { obsClearErr(); Console::print("误差统计已清零\r\n"); }

static void obsReport() {
  if (g_errN == 0) { Console::print("还没有有效样本（转速太低？）\r\n"); return; }
  const float mean = g_errSum / g_errN;
  const float var  = g_errSum2 / g_errN - mean * mean;
  const float std  = sqrtf(var > 0 ? var : 0);
  const float we   = fabsf(g_m.omegaE);
  Console::printfln("---- 观测器误差（真值=AS5600）@ ωe=%.1f rad/s (%.0f rpm 机械) ----", we,
                    g_spdRef * 60.0f / TWO_PI_F);
  Console::printfln("平均误差 = %+.2f°   标准差 = %.2f°   峰值 = %.2f°   (样本 %d)", mean, std,
                    g_errPeak, g_errN);
  Console::printfln("观测器估计 ωe = %.1f rad/s  (真值 %.1f)  比值 %.3f", g_obs.omega(), we,
                    g_obs.omega() / (we + 1e-6f));
}

static void cmdReport(int, char**) { obsReport(); }

/// 取一段稳态数据并打印统计（非阻塞版：内部用 obsWait）
static void obsMeasure(uint32_t ms) {
  obsClearErr();
  obsWait(ms);
}

/**
 * 自动扫转速 → 打印"角度误差 vs 转速"曲线（E4.7 的核心交付物）
 * 非阻塞：等待期间由 obsWait→obsServiceOnce 继续跑控制
 */
static void cmdSweepSpd(int argc, char** argv) {
  const float from = Console::argf(argv, argc, 1, 5.0f);
  const float to   = Console::argf(argv, argc, 2, 60.0f);
  const float stp  = Console::argf(argv, argc, 3, 5.0f);
  if (!g_running) { Console::print("先 start\r\n"); return; }

  Console::print("ωm(rad/s)  ωe(rad/s)   rpm    误差均值(°)  误差std(°)  误差峰值(°)  估计ωe/真值\r\n");
  for (float w = from; w <= to + 1e-3f; w += stp) {
    g_spdRef = w;
    obsWait(1200);            // 等转速稳定（期间控制照跑）
    obsMeasure(800);          // 采样窗口
    const float we = fabsf(g_m.omegaE);
    if (g_errN < 5) {
      Console::printfln("%8.2f   %8.2f   %5.0f   —— 样本不足（反 EMF 太小）", w, we,
                        w * 60.0f / TWO_PI_F);
      continue;
    }
    const float mean = g_errSum / g_errN;
    const float var  = g_errSum2 / g_errN - mean * mean;
    Console::printfln("%8.2f   %8.2f   %5.0f   %+10.2f  %10.2f  %10.2f  %8.3f", w, we,
                      w * 60.0f / TWO_PI_F, mean, sqrtf(var > 0 ? var : 0), g_errPeak,
                      g_obs.omega() / (we + 1e-6f));
  }
  Console::print("→ 这就是 12.3 要的那条【角度误差 vs 转速】曲线\r\n");
}

// ============================================================================
//  框架
// ============================================================================

static void obsBegin(float fsw = 20000.0f) {
  telemetry::begin(OBS_BAUD);
  g_con.begin(OBS_BAUD);
  delay(300);

  FocConfig c;
  c.fsw = fsw;
  c.currentLimit = 1.0f;
  g_m.begin(c);
  g_m.setLoopHz(5000.0f);
  g_m.designCurrentLoop(TWO_PI_F * 300.0f);
  obsDesignSpeed(g_wcsHz);
  g_obs.reset();

  g_con.add("align",    cmdAlign,    "对齐并清零编码器");
  g_con.add("start",    cmdStart,    "启动（控制用编码器）");
  g_con.add("stop",     cmdStop,     "停止");
  g_con.add("spd",      cmdSpd,      "spd <rad/s> 机械转速给定");
  g_con.add("wcs",      cmdWcs,      "wcs <Hz> 速度环带宽");
  g_con.add("errc",     cmdErrClear, "清零误差统计");
  g_con.add("report",   cmdReport,   "打印观测器误差");
  g_con.add("sweepspd", cmdSweepSpd, "sweepspd <from> <to> <step> 自动扫转速出误差曲线");
}

static void obsLoop() {
  g_con.poll();
  obsServiceOnce();
  telemetry::set(0, g_m.thetaE * 180.0f / PI_F);
  telemetry::set(1, g_obs.theta() * 180.0f / PI_F);
  telemetry::set(2, wrapPi(g_obs.theta() - g_m.thetaE) * 180.0f / PI_F);
  telemetry::set(3, g_m.omegaE);
  telemetry::set(4, g_obs.omega());
  telemetry::set(5, g_obs.bemfMag());
  telemetry::tick();
}

static void obsAddTelemetryNames() {
  telemetry::setChannels(6);
  telemetry::setChannelName(0, "theta_true");
  telemetry::setChannelName(1, "theta_hat");
  telemetry::setChannelName(2, "err_deg");
  telemetry::setChannelName(3, "omega_true");
  telemetry::setChannelName(4, "omega_hat");
  telemetry::setChannelName(5, "bemf_mag");
}
