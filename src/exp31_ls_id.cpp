/**
 * E5.2 电感辨识（阶段 5）
 *
 * 目标：测量 Ld、Lq，并测凸极率。
 * 验收：得到 Ld、Lq；测出云台电机的凸极率（预期 1.01~1.05）。
 * ★ 副产品：凸极率数据直接证明"为什么 HFI 在这台电机上不可用"。
 *
 * 两种方法（本工程都实现了）：
 *   ① 阶跃法（推荐给 2208 这类高电感电机）：
 *        电压模式在 d 轴上给一个电压阶跃 → 电流按 i(t)=I∞(1−e^{−t/τ}) 上升
 *        测出 τ（到 63.2% 的时间）→ L = τ·R
 *   ② 高频方波法（适合低电感航模电机）：
 *        方波 ±V、频率 f → 半周期内电流近似线性 → ΔI = V/(2f·L) → L = V/(2f·ΔI)
 *
 * ⚠ 必须先测好 R（E5.1），否则 L 会按比例错。本实验用 g_obs.rs 作为 R。
 *
 * 串口命令：
 *   axis d|q         注入轴（转子锁住时 d/q 由编码器角度决定）
 *   v <V>            阶跃/方波电压幅值
 *   step             阶跃法测 L
 *   hf <Hz>          高频方波法测 L（给频率）
 *   record           把当前 (θe, L) 记入表
 *   table            打印 L(θ) 表并提取 Ld/Lq/凸极率
 *   r <ohm>          设定 R
 */

#include <Arduino.h>
#include "obs_common.h"

static char  g_axis = 'd';
static float g_v = 2.0f;
static float g_rs = 10.0f;

struct LRec { float thetaDeg; float L; };
static const int MAXREC = 36;
static LRec g_rec[MAXREC];
static int  g_nrec = 0;

static void setInjection(float v) {
  g_m.voltageMode = true;
  g_m.useExternalAngle = false;     // 用编码器角度（转子锁住时它就是真实的 d/q 方向）
  if (g_axis == 'd') { g_m.vdRef = v; g_m.vqRef = 0.0f; }
  else               { g_m.vdRef = 0.0f; g_m.vqRef = v; }
  g_m.enable(true);
  g_running = true;
}

static void stopInjection() {
  g_m.vdRef = g_m.vqRef = 0.0f;
  g_m.voltageMode = false;
  g_m.enable(false);
  g_running = false;
}

static void cmdAxis(int argc, char** argv) {
  if (argc >= 2) g_axis = argv[1][0];
  Console::printfln("注入轴 = %c", g_axis);
}
static void cmdV(int argc, char** argv) {
  g_v = Console::argf(argv, argc, 1, g_v);
  Console::printfln("注入电压 = %.2f V", g_v);
}
static void cmdR(int argc, char** argv) {
  g_rs = Console::argf(argv, argc, 1, g_rs);
  g_obs.rs = g_rs;
  Console::printfln("R = %.3f Ω", g_rs);
}

/// ① 阶跃法
static void cmdStep(int, char**) {
  Console::print("阶跃法：先归零，再给电压阶跃，采样电流上升曲线\r\n");
  // 先归零
  stopInjection();
  setInjection(0.0f);
  { const uint32_t t0 = millis(); while (millis() - t0 < 300) { obsServiceOnce(); delay(1); } }

  // 施加阶跃，采样
  setInjection(g_v);
  const int N = 48;
  float ibuf[N];
  const uint32_t t0 = micros();
  uint32_t tbuf[N];
  for (int k = 0; k < N; k++) {
    obsServiceOnce();
    tbuf[k] = micros();
    ibuf[k] = (g_axis == 'd') ? g_m.id : g_m.iq;
    delayMicroseconds(180);
  }
  stopInjection();

  // 终值取最后 8 点的平均
  float iInf = 0;
  for (int k = N - 8; k < N; k++) iInf += ibuf[k];
  iInf /= 8.0f;
  if (fabsf(iInf) < 1e-3f) { Console::print("电流太小，测不出（加大 v 或先 align）\r\n"); return; }

  // 找 63.2% 时刻
  const float target = 0.632f * iInf;
  float tau = -1.0f;
  for (int k = 1; k < N; k++) {
    if ((ibuf[k - 1] < target && ibuf[k] >= target) ||
        (ibuf[k - 1] > target && ibuf[k] <= target)) {
      const float t0s = (float)(tbuf[k - 1] - t0) * 1e-6f;
      const float t1s = (float)(tbuf[k] - t0) * 1e-6f;
      const float d0 = ibuf[k - 1] - target, d1 = ibuf[k] - target;
      const float frac = (fabsf(d1 - d0) > 1e-9f) ? fabsf(d0) / fabsf(d1 - d0) : 0.0f;
      tau = t0s + frac * (t1s - t0s);
      break;
    }
  }
  if (tau <= 0) { Console::print("没找到 63.2% 点（阶跃太快或太慢，换个 v 或轴）\r\n"); return; }

  const float L = tau * g_rs;
  Console::printfln("I∞ = %.4f A   τ = %.3f ms   →  L%c = %.5f H (%.2f mH)", iInf, tau * 1000.0f,
                    g_axis, L, L * 1000.0f);
  Console::printfln("（校验：τ·R = %.3f ms × %.2f Ω = %.2f mH）", tau * 1000.0f, g_rs, L * 1000.0f);
}

/// ② 高频方波法
static void cmdHf(int argc, char** argv) {
  const float f = Console::argf(argv, argc, 1, 2000.0f);
  Console::printfln("高频方波法：f = %.0f Hz，半周期 %.0f µs", f, 5e5f / f);

  setInjection(0.0f);
  { const uint32_t t0 = millis(); while (millis() - t0 < 300) { obsServiceOnce(); delay(1); } }

  // 手工按半周期翻转电压，同时记录电流极值
  const uint32_t halfUs = (uint32_t)(5e5f / f);
  float iMax = -1e9f, iMin = 1e9f;
  for (int k = 0; k < 6; k++) {           // 3 个完整周期
    setInjection((k % 2 == 0) ? g_v : -g_v);
    const uint32_t t0 = micros();
    while (micros() - t0 < halfUs) {
      obsServiceOnce();
      const float i = (g_axis == 'd') ? g_m.id : g_m.iq;
      if (i > iMax) iMax = i;
      if (i < iMin) iMin = i;
      delayMicroseconds(50);
    }
  }
  stopInjection();
  const float dI = iMax - iMin;
  if (dI < 1e-3f) { Console::print("电流纹波太小，测不出（提高 v 或降低 f）\r\n"); return; }
  const float L = g_v / (2.0f * f * dI);
  Console::printfln("ΔI = %.4f A  →  L%c = %.5f H (%.2f mH)", dI, g_axis, L, L * 1000.0f);
}

static void cmdRecord(int, char**) {
  if (g_nrec >= MAXREC) { Console::print("表已满\r\n"); return; }
  // 用阶跃法测一次并记录（复用 cmdStep 的逻辑会打印太多，这里做简化版）
  const float savedV = g_v;
  stopInjection();
  setInjection(0.0f);
  { const uint32_t t0 = millis(); while (millis() - t0 < 200) { obsServiceOnce(); delay(1); } }
  setInjection(savedV);
  const int N = 48;
  float ibuf[N]; uint32_t tbuf[N];
  const uint32_t t0 = micros();
  for (int k = 0; k < N; k++) {
    obsServiceOnce();
    tbuf[k] = micros();
    ibuf[k] = (g_axis == 'd') ? g_m.id : g_m.iq;
    delayMicroseconds(180);
  }
  const float thDeg = g_m.thetaE * 180.0f / PI_F;
  stopInjection();

  float iInf = 0;
  for (int k = N - 8; k < N; k++) iInf += ibuf[k];
  iInf /= 8.0f;
  if (fabsf(iInf) < 1e-3f) { Console::print("测不出\r\n"); return; }
  const float target = 0.632f * iInf;
  float tau = -1;
  for (int k = 1; k < N; k++) {
    if ((ibuf[k - 1] < target && ibuf[k] >= target) || (ibuf[k - 1] > target && ibuf[k] <= target)) {
      const float t0s = (float)(tbuf[k - 1] - t0) * 1e-6f, t1s = (float)(tbuf[k] - t0) * 1e-6f;
      const float d0 = ibuf[k - 1] - target, d1 = ibuf[k] - target;
      tau = t0s + (fabsf(d1 - d0) > 1e-9f ? fabsf(d0) / fabsf(d1 - d0) : 0.0f) * (t1s - t0s);
      break;
    }
  }
  if (tau <= 0) { Console::print("测不出 τ\r\n"); return; }
  g_rec[g_nrec].thetaDeg = thDeg;
  g_rec[g_nrec].L = tau * g_rs;
  g_nrec++;
  Console::printfln("记录 #%d: θe=%.1f°  L=%.2f mH", g_nrec, thDeg, tau * g_rs * 1000.0f);
}

static void cmdTable(int, char**) {
  if (g_nrec < 3) { Console::print("记录太少（用 record 在不同转子位置各测一次）\r\n"); return; }
  float Lmin = 1e9f, Lmax = -1e9f, thMin = 0, thMax = 0;
  Console::print("  θe(°)     L(mH)\r\n");
  for (int i = 0; i < g_nrec; i++) {
    Console::printfln("%7.1f   %7.2f", g_rec[i].thetaDeg, g_rec[i].L * 1000.0f);
    if (g_rec[i].L < Lmin) { Lmin = g_rec[i].L; thMin = g_rec[i].thetaDeg; }
    if (g_rec[i].L > Lmax) { Lmax = g_rec[i].L; thMax = g_rec[i].thetaDeg; }
  }
  Console::printfln("Ld ≈ %.2f mH @ %.0f°   Lq ≈ %.2f mH @ %.0f°", Lmin * 1000.0f, thMin,
                    Lmax * 1000.0f, thMax);
  Console::printfln("凸极率 Lq/Ld = %.4f", Lmax / fmaxf(Lmin, 1e-6f));
  Console::print("→ 凸极率 ≈1.0x 说明这是表贴式（SPM）电机 → MTPA/HFI 都不适用（这正是要证明的）\r\n");
}

void setup() {
  obsBegin();
  g_m.setLoopHz(10000.0f);          // 阶跃法需要更密的采样
  Console::print("\r\n=== E5.2 电感辨识（阶跃法 + 高频方波法）===\r\n");
  Console::print("★ 请锁住转子；流程：align → r 10 → v 2 → axis d → step\r\n");
  Console::print("   测 Ld/Lq：把转子拨到若干不同位置，每个位置 record 一次，最后 table\r\n");

  g_con.add("axis",   cmdAxis,   "axis d|q");
  g_con.add("v",      cmdV,      "v <V> 注入电压");
  g_con.add("r",      cmdR,      "r <ohm> 设定 R");
  g_con.add("step",   cmdStep,   "阶跃法测 L");
  g_con.add("hf",     cmdHf,     "hf <Hz> 高频方波法");
  g_con.add("record", cmdRecord, "记录当前转子位置的 L");
  g_con.add("table",  cmdTable,  "打印 L(θ) 表并提取 Ld/Lq");
  obsAddTelemetryNames();
}

void loop() { obsLoop(); }
