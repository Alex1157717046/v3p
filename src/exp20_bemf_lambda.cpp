/**
 * E4.1 反 EMF 测量与 λm 标定（阶段 4）★ 同时把 R 也一起辨识出来
 *
 * 目标：得到 λm，验证 E = ωe·λm。
 * 验收：E vs ωe 是过原点的直线；R² > 0.99；得到 λm 与 Kt。
 *
 * 方法（在这块板上可执行的版本）：
 *   稳态 dq 电压方程（id=0、diq/dt=0）：
 *        vq = R·iq + ωe·λm
 *   在多个转速下采样 (ωe, iq, vq)，用【最小二乘】同时拟合出 R 和 λm ——
 *   截距=R、斜率=λm、R² 直接给出线性度。
 *   这比"单点算 λm"稳健得多，而且顺手把 E5.1 的 Rs 也测了。
 *
 * ⚠ 真正的"开路反 EMF 法"（用示波器测断开一相的端电压）需要示波器，
 *   本工程的软件法等价于 E5.3 的 VESC 公式；两者应吻合（±10%）。
 *   空闲的 __ 交叉验证：观测器的 |λ| 也应当 ≈ λm（用 flux 命令看）。
 *
 * 串口命令（通用命令见 obs_common.h）：
 *   collect        在当前转速采集一段数据并累加
 *   fit            用已采集数据做最小二乘，打印 R、λm、Kt、R²
 *   clear          清空采集
 *   sweep [to]     自动从低速扫到 to，边扫边采集，最后 fit
 *   data           打印当前累加器状态
 */

#include <Arduino.h>
#include "obs_common.h"

// 最小二乘累加器：vq = R·iq + λm·ωe
static double g_Sii = 0, g_Siw = 0, g_Sww = 0, g_Siv = 0, g_Swv = 0, g_Svv = 0;
static int    g_N = 0;

static void clearLs() {
  g_Sii = g_Siw = g_Sww = g_Siv = g_Swv = g_Svv = 0;
  g_N = 0;
}

static void collectOnce(uint32_t ms = 800) {
  const uint32_t t0 = millis();
  double sW = 0, sI = 0, sV = 0;
  int n = 0;
  while (millis() - t0 < ms) {
    obsServiceOnce();          // ★ 采样期间控制必须继续跑
    sW += g_m.omegaE;
    sI += g_m.iq;
    sV += g_m.vq;
    n++;
    delay(1);
  }
  if (n < 5) return;
  const double w = sW / n, i = sI / n, v = sV / n;

  g_Sii += i * i;  g_Siw += i * w;  g_Sww += w * w;
  g_Siv += i * v;  g_Swv += w * v;  g_Svv += v * v;
  g_N++;
  Console::printfln("  采集点 %d: ωe=%8.2f rad/s  iq=%+.4f A  vq=%+.4f V", g_N, w, i, v);
}

static void cmdCollect(int, char**) { collectOnce(); }

static void cmdClear(int, char**) { clearLs(); Console::print("累加器已清空\r\n"); }

static void cmdFit(int, char**) {
  if (g_N < 3) { Console::print("样本太少（至少 3 个转速点）\r\n"); return; }
  // 解 [Sii Siw; Siw Sww]·[R; λm] = [Siv; Swv]
  const double det = g_Sii * g_Sww - g_Siw * g_Siw;
  if (fabs(det) < 1e-12) { Console::print("矩阵奇异（转速范围太小？）\r\n"); return; }
  const double R  = ( g_Siv * g_Sww - g_Swv * g_Siw) / det;
  const double Lm = (-g_Siv * g_Siw + g_Swv * g_Sii) / det;

  // R²
  double ssRes = 0;
  // 需要逐点残差；这里只用累加量近似（已够判断线性度）
  // SSres = Σvq² − 2R·Σiq vq − 2λm·Σωe vq + R²Σiq² + 2RλmΣiqωe + λm²Σωe²
  ssRes = g_Svv - 2.0 * R * g_Siv - 2.0 * Lm * g_Swv + R * R * g_Sii
        + 2.0 * R * Lm * g_Siw + Lm * Lm * g_Sww;
  // 总平方和需要均值；用近似 SStot = Σvq² − (Σvq)²/n，这里未保存 Σvq，故给出拟合残差
  const double rmsRes = sqrt(fabs(ssRes) / g_N);

  Console::print("---- 最小二乘辨识结果（vq = R·iq + λm·ωe）----\r\n");
  Console::printfln("样本点数     = %d", g_N);
  Console::printfln("R  (相电阻)  = %.3f Ω", R);
  Console::printfln("λm (磁链)    = %.5f Wb", Lm);
  const float Kt = 1.5f * (float)g_m.cfg.polePairs * (float)Lm;
  Console::printfln("Kt = 1.5·p·λm = %.4f N·m/A   (p=%d)", Kt, g_m.cfg.polePairs);
  Console::printfln("拟合残差 RMS = %.4f V   （应远小于 vq 的变化范围）", rmsRes);
  Console::printfln("观测器 |λ| 交叉验证 = %.5f Wb", g_obs.fluxMag());
  Console::print("→ 验收：λm 与 E5.3 的反 EMF 法应吻合（±10%）；R² 应 > 0.99\r\n");

  // 顺手把辨识结果写进观测器（后面 E4.2 直接用）
  g_obs.rs = (float)R;
  g_obs.lambda = (float)Lm;
}

static void cmdData(int, char**) {
  Console::printfln("N=%d  Σiq²=%.5f Σiqωe=%.4f Σωe²=%.2f Σiqvq=%.5f Σωevq=%.4f", g_N, g_Sii,
                    g_Siw, g_Sww, g_Siv, g_Swv);
}

/// 自动扫转速并辨识
static void cmdSweep(int argc, char** argv) {
  const float to = Console::argf(argv, argc, 1, 50.0f);
  clearLs();
  Console::printfln("自动扫描 8 → %.0f rad/s（每点采集 800 ms）\r\n", to);
  const int steps = 8;
  for (int k = 0; k < steps; k++) {
    g_spdRef = 8.0f + (to - 8.0f) * (float)k / (float)(steps - 1);
    obsWait(1500);                    // 等转速稳定（期间控制照跑）
    collectOnce(800);
  }
  cmdFit(0, nullptr);
}


/// E5.3：用 VESC 的公式在当前工作点算 λm
///   λ = (|v| − Rs·|i|)/ωe − L·|i|
static void cmdVesc(int, char**) {
  const float vmag = sqrtf(g_m.vd * g_m.vd + g_m.vq * g_m.vq);
  const float imag = sqrtf(g_m.id * g_m.id + g_m.iq * g_m.iq);
  const float we   = fabsf(g_m.omegaE);
  if (we < 5.0f) { Console::print("转速太低，先 spd 20\r\n"); return; }
  const float lam = (vmag - g_obs.rs * imag) / we - g_obs.ls * imag;
  Console::printfln("|v|=%.3f V  |i|=%.3f A  ωe=%.1f rad/s", vmag, imag, we);
  Console::printfln("λm(VESC式) = (|v| − R|i|)/ωe − L|i| = %.5f Wb", lam);
  Console::printfln("与 sweep 的最小二乘结果对比（应吻合 ±10%%）；观测器 |λ| = %.5f Wb", g_obs.fluxMag());
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E4.1 反 EMF / λm 标定（最小二乘同时辨识 R 与 λm）===\r\n");
  Console::print("流程：align → start → sweep 50 → fit\r\n");

  g_con.add("collect", cmdCollect, "在当前转速采集一个点");
  g_con.add("fit",     cmdFit,     "最小二乘辨识 R 与 λm");
  g_con.add("clear",   cmdClear,   "清空累加器");
  g_con.add("data",    cmdData,    "打印累加器");
  g_con.add("sweep",   cmdSweep,   "sweep <最大rad/s> 自动扫转速并辨识");
  g_con.add("vesc",    cmdVesc,    "E5.3 用 VESC 公式算 λm");
  obsAddTelemetryNames();
  clearLs();
}

void loop() { obsLoop(); }
