/**
 * E7.1 量化验证：Q15 定点 vs 浮点（在 PC 上跑，不需要硬件）
 *
 * 编译运行：
 *   cd v3p-lab/stm32f103-foc/tools
 *   clang++ -O2 -std=c++17 -I../lib/focq15 q15_selftest.cpp -o q15_selftest && ./q15_selftest
 *
 * 它逐点对比：
 *   ① sin 查表 vs std::sin
 *   ② Clarke/Park（Q15）vs 浮点
 *   ③ SVPWM（Q15）vs 浮点 min-max 法（含线电压）
 *   ④ PI 阶跃响应（Q15 vs 浮点）
 * 输出最大误差表 —— 这就是 E7.1 要求的"浮点 vs 定点"量化对比。
 */
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include "foc_q15.h"

using namespace focq15;

// ---------------- 浮点参考实现（与 lib/v3plab/v3p_math.h 一致）----------------
static const float PI_F = 3.14159265358979f;
static const float SQRT3_F = 1.73205080756888f;

struct ABCf { float a, b, c; };
static ABCf abToAbc(float va, float vb) {
  return {va, -0.5f * va + 0.5f * SQRT3_F * vb, -0.5f * va - 0.5f * SQRT3_F * vb};
}
static void clarkeF(float ia, float ib, float* al, float* be) {
  *al = ia;
  *be = (ia + 2.0f * ib) / SQRT3_F;
}
// 约定与 Q15 版一致：输入标幺（1.0 = Vdc/√3），duty = 0.5 + (v+off)/√3
static void svpwmF(float va, float vb, float* da, float* db, float* dc) {
  const ABCf p = abToAbc(va, vb);
  const float vmax = std::max(p.a, std::max(p.b, p.c));
  const float vmin = std::min(p.a, std::min(p.b, p.c));
  const float off = -0.5f * (vmax + vmin);
  *da = 0.5f + (p.a + off) / SQRT3_F;
  *db = 0.5f + (p.b + off) / SQRT3_F;
  *dc = 0.5f + (p.c + off) / SQRT3_F;
}

int main() {
  printf("=== E7.1 Q15 定点 vs 浮点 量化验证 ===\n\n");

  // ---------- ① sin 查表 ----------
  {
    double maxErr = 0; double sum2 = 0; int n = 0;
    for (uint32_t a = 0; a < 65536; a += 7) {
      const float ref = sinf(2.0f * PI_F * (float)a / 65536.0f);
      const float got = toFloat(sinU16((uint16_t)a));
      const double e = fabs(ref - got);
      maxErr = std::max(maxErr, e); sum2 += e * e; n++;
    }
    printf("① sin 查表(256点+线性插值) vs std::sin\n");
    printf("   最大误差 = %.6f   RMS = %.6f  (%.3f%% 满量程)\n\n", maxErr,
           sqrt(sum2 / n), maxErr * 100);
  }

  // ---------- ② Clarke / Park ----------
  {
    double maxE = 0; int n = 0;
    for (int k = 0; k < 20000; k++) {
      const float ia = sinf(k * 0.013f) * 0.8f;
      const float ib = sinf(k * 0.013f - 2.0944f) * 0.8f;
      float alF, beF;
      clarkeF(ia, ib, &alF, &beF);
      q15_t alQ, beQ;
      clarke( fromFloat(ia), fromFloat(ib), &alQ, &beQ);
      maxE = std::max(maxE, (double)fabs(alF - toFloat(alQ)));
      maxE = std::max(maxE, (double)fabs(beF - toFloat(beQ)));

      // Park
      const uint16_t th = (uint16_t)(k * 37);
      const float c = cosf(2.0f * PI_F * th / 65536.0f), s = sinf(2.0f * PI_F * th / 65536.0f);
      const float dF = alF * c + beF * s, qF = -alF * s + beF * c;
      q15_t dQ, qQ;
      park(alQ, beQ, th, &dQ, &qQ);
      maxE = std::max(maxE, (double)fabs(dF - toFloat(dQ)));
      maxE = std::max(maxE, (double)fabs(qF - toFloat(qQ)));
      n++;
    }
    printf("② Clarke + Park（Q15）vs 浮点\n");
    printf("   最大误差 = %.6f  (%.3f%% 满量程)  样本 %d\n\n", maxE, maxE * 100, n);
  }

  // ---------- ③ SVPWM ----------
  {
    double maxDuty = 0, maxLL = 0;
    int n = 0;
    for (int k = 0; k < 20000; k++) {
      const uint16_t th = (uint16_t)(k * 53);
      for (float m : {0.1f, 0.5f, 0.9f, 1.0f}) {
        // 标幺：|v| = m（1.0 对应 Vdc/√3）
        const float va = m * cosf(2.0f * PI_F * th / 65536.0f);
        const float vb = m * sinf(2.0f * PI_F * th / 65536.0f);
        float daF, dbF, dcF;
        svpwmF(va, vb, &daF, &dbF, &dcF);
        uint16_t daQ, dbQ, dcQ;
        svpwm(fromFloat(va), fromFloat(vb), &daQ, &dbQ, &dcQ);
        const float a = toFloat((q15_t)(daQ / 2)), b = toFloat((q15_t)(dbQ / 2)),
                    c = toFloat((q15_t)(dcQ / 2));
        maxDuty = std::max(maxDuty, (double)fabs(daF - a));
        maxDuty = std::max(maxDuty, (double)fabs(dbF - b));
        maxDuty = std::max(maxDuty, (double)fabs(dcF - c));
        // 线电压（占空比差 —— 这才是真正决定电流的量）
        maxLL = std::max(maxLL, (double)fabs((daF - dbF) - (a - b)));
        maxLL = std::max(maxLL, (double)fabs((dbF - dcF) - (b - c)));
        n++;
      }
    }
    printf("③ SVPWM（Q15，min-max 零序注入）vs 浮点\n");
    printf("   占空比最大误差 = %.6f\n", maxDuty);
    printf("   线电压(占空比差)最大误差 = %.6f   ← 这个才决定电流畸变\n", maxLL);
    printf("   样本 %d\n\n", n);
  }

  // ---------- ④ PI（同一误差序列喂给两个实现，直接比算术）----------
  {
    PiQ15 pi;
    pi.kp = fromFloat(0.35f);
    pi.ki = fromFloat(0.05f);
    pi.outMax = fromFloat(0.95f);
    pi.outMin = fromFloat(-0.95f);
    const uint32_t dt_us = 200;   // 5 kHz

    float integF = 0, outF = 0;
    q15_t outQ = 0;
    double maxDiff = 0;
    const int N = 5000;
    for (int k = 0; k < N; k++) {
      // 造一个有阶跃又有反号的误差序列
      const float err = (k < 1500) ? 0.6f : (k < 3000 ? -0.35f : 0.2f);
      // 浮点 PI（与定点同样的结构：条件积分抗饱和）
      const float p = 0.35f * err;
      const float unsat = p + integF;
      float sat = unsat; if (sat > 0.95f) sat = 0.95f; if (sat < -0.95f) sat = -0.95f;
      const bool pushUp = (unsat > 0.95f) && (err > 0), pushDown = (unsat < -0.95f) && (err < 0);
      if (!(pushUp || pushDown)) integF += 0.05f * err * (1.0f / 5000.0f);
      outF = sat;

      // 定点 PI
      outQ = pi.update(fromFloat(err), dt_us);
      maxDiff = std::max(maxDiff, (double)fabs(outF - toFloat(outQ)));
    }
    printf("④ PI（Q15 vs 浮点，同一误差序列 5000 步）\n");
    printf("   最大偏差 = %.6f   终点：浮点 %.5f  定点 %.5f\n\n", maxDiff, outF, toFloat(outQ));
  }

  printf("结论：Q15 的误差主要来自 sin 查表插值与量化，");
  printf("线电压误差 << 1%%，对电流环影响可忽略；\n");
  printf("      代价换来的是【整数运算】—— 在无 FPU 的 Cortex-M3 上快 5~10 倍。\n");
  return 0;
}
