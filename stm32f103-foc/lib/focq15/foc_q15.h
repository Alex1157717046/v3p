/**
 * @file foc_q15.h
 * @brief Q15 定点 FOC 数学库（E7.1 定点化移植的核心）
 *
 * 设计要点（对应 12.6 E7.1 的"坑"）：
 *   · Q15 = 1 符号 + 15 小数位，范围 [−1, 1)
 *   · Q15×Q15 得 Q30，必须右移 15 位；累加要用 Q31 或 int64
 *   · 所有中间结果用【饱和运算】，否则电流一大就跳变/发散
 *   · sin/cos 用 256 点表 + 线性插值（用象限对称性省 3/4 表空间）
 *   · PI 的积分器用 Q31（位宽足够），并做抗饱和
 *
 * 本文件**不依赖任何硬件/Arduino**，所以可以在 PC 上直接编译做数值对比
 * （见 tools/q15_selftest.cpp）—— 这就是 E7.1 要求的"浮点 vs 定点"量化验证。
 */
#pragma once

#include <stdint.h>

namespace focq15 {

typedef int16_t q15_t;
typedef int32_t q31_t;

constexpr q15_t Q15_MAX =  32767;
constexpr q15_t Q15_MIN = -32768;

// ============================ 基本运算 ============================

/// 饱和到 int16
static inline q15_t sat16(q31_t x) {
  if (x > Q15_MAX) return Q15_MAX;
  if (x < Q15_MIN) return Q15_MIN;
  return (q15_t)x;
}

/// Q15 × Q15 → Q15（含饱和）
static inline q15_t mul(q15_t a, q15_t b) {
  return sat16(((q31_t)a * (q31_t)b) >> 15);
}

/// Q15 × Q15 → Q30（不右移，供累加用）
static inline q31_t mul_q30(q15_t a, q15_t b) {
  return (q31_t)a * (q31_t)b;
}

/// 饱和到 q31
static inline q31_t sat31(int64_t x) {
  if (x > 2147483647LL) return 2147483647;
  if (x < -2147483648LL) return (q31_t)(-2147483647LL - 1);
  return (q31_t)x;
}

/// float → Q15（供标定用，运行时不调用）
static inline q15_t fromFloat(float x) {
  if (x >= 1.0f) return Q15_MAX;
  if (x <= -1.0f) return Q15_MIN;
  return (q15_t)(x * 32768.0f);
}
static inline float toFloat(q15_t x) { return (float)x / 32768.0f; }

// ============================ sin / cos 查表 ============================
//
// 角度用 uint16 表示：0~65535 对应 0~2π（即 Q16 的弧度/2π）
// 表只存 0~π/2 的 65 个点（整数 + 每 1/64 象限一个插值点），用象限对称展开。

static const q15_t kSinQuarter[65] = {
      0,   804,  1608,  2410,  3212,  4011,  4808,  5602,  6393,  7179,
   7962,  8739,  9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732,
  15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403,
  22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
  27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571,
  30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521,
  32609, 32678, 32728, 32757, 32767
};

/// 输入 0~65535（= 0~2π），输出 Q15 的正弦
static inline q15_t sinU16(uint16_t a) {
  const uint16_t quad = a >> 14;          // 0..3
  uint16_t idx = a & 0x3FFF;              // 象限内 0..16383
  if (quad & 1) idx = 16384 - idx;        // 2、4 象限镜像
  // idx(0..16384) → 表下标 0..64，用 10 位精度做线性插值
  const uint16_t pos = (uint16_t)((uint32_t)idx * 64u / 16384u);   // 0..64
  const uint16_t frac = (uint16_t)(((uint32_t)idx * 64u) % 16384u); // 0..16383
  const q15_t s0 = kSinQuarter[pos > 64 ? 64 : pos];
  const q15_t s1 = kSinQuarter[(pos + 1) > 64 ? 64 : (pos + 1)];
  const q15_t v = (q15_t)(s0 + (q15_t)(((q31_t)(s1 - s0) * (q31_t)frac) >> 14));
  return (quad >= 2) ? (q15_t)(-v) : v;
}

/// 余弦 = 正弦移相 90°
static inline q15_t cosU16(uint16_t a) { return sinU16((uint16_t)(a + 16384)); }

// ============================ Clarke / Park ============================
//
// 为方便 Q15，这里把电流标幺化到【额定电流】：
//   i_pu = i / I_base，I_base 由使用者在标定时确定（如 2 A）
// 变换系数用 Q15 表示：1/√3 = 0.57735 → 18919

constexpr q15_t ONE_BY_SQRT3_Q15 = 18919;

/// iα = ia ;  iβ = (ia + 2·ib)/√3
/// ⚠ 关键：平衡三相时 (ia+2ib) 可达 √3×幅值，会超出 Q15 —— 所以中间量必须用
///   32/64 位承载，只在最后一步饱和。（这是定点 Clarke 最经典的溢出坑）
static inline void clarke(q15_t ia, q15_t ib, q15_t* alpha, q15_t* beta) {
  *alpha = ia;
  const int64_t t = (int64_t)ia + 2 * (int64_t)ib;      // Q15, 可到 ±3×
  *beta = sat16((q31_t)((t * ONE_BY_SQRT3_Q15) >> 15));
}

/// id =  iα·cosθ + iβ·sinθ ;  iq = −iα·sinθ + iβ·cosθ
static inline void park(q15_t alpha, q15_t beta, uint16_t theta, q15_t* d, q15_t* q) {
  const q15_t c = cosU16(theta), s = sinU16(theta);
  *d = sat16((q31_t)mul(alpha, c) + (q31_t)mul(beta, s));
  *q = sat16((q31_t)mul(beta, c) - (q31_t)mul(alpha, s));
}

/// 反 Park
static inline void invPark(q15_t d, q15_t q, uint16_t theta, q15_t* alpha, q15_t* beta) {
  const q15_t c = cosU16(theta), s = sinU16(theta);
  *alpha = sat16((q31_t)mul(d, c) - (q31_t)mul(q, s));
  *beta  = sat16((q31_t)mul(d, s) + (q31_t)mul(q, c));
}

// ============================ SVPWM（min-max 零序注入法）============================
//
// 与浮点版 svpwmMinMax 等价，但全定点。
// 输入：αβ 电压（标幺，1.0 = Vdc/√3）；输出：三相占空比 0~65535

constexpr q15_t SQRT3_2_Q15 = 28378;   // √3/2 = 0.86603

/**
 * 输入：αβ 电压，标幺约定 【1.0 = Vdc/√3】（即 SVPWM 线性区极限）
 *   → duty_x = 0.5 + (v_x + off)/√3
 *   这样 vdc 已经被"归一化掉了"，不需要再传母线电压，也就不会出现
 *   "vdc>1 被 Q15 饱和"这类标度事故（★ 这是定点化最经典的坑之一）。
 * 输出：三相占空比 0~65535
 */
static inline void svpwm(q15_t va, q15_t vb, uint16_t* da, uint16_t* db, uint16_t* dc) {
  const q15_t vA = va;
  const q15_t vB = sat16(-(q31_t)va / 2 + mul(vb, SQRT3_2_Q15));
  const q15_t vC = sat16(-(q31_t)va / 2 - mul(vb, SQRT3_2_Q15));

  const q15_t vmax = vA > vB ? (vA > vC ? vA : vC) : (vB > vC ? vB : vC);
  const q15_t vmin = vA < vB ? (vA < vC ? vA : vC) : (vB < vC ? vB : vC);
  const q15_t off  = (q15_t)(-((q31_t)vmax + (q31_t)vmin) / 2);   // 零序注入

  auto duty = [&](q15_t p) -> uint16_t {
    const q31_t q = mul(sat16((q31_t)p + off), ONE_BY_SQRT3_Q15);  // (v+off)/√3
    q31_t r = 16384 + q;                                           // 0.5 + ...
    if (r < 0) r = 0;
    if (r > 32767) r = 32767;
    return (uint16_t)((uint32_t)r * 2u);                           // 0..32767 → 0..65534
  };
  *da = duty(vA);
  *db = duty(vB);
  *dc = duty(vC);
}

// ============================ PI（Q31 积分器 + 抗饱和）============================

struct PiQ15 {
  q15_t kp = 0, ki = 0;              // ki 的单位：1/s（Q15）
  q15_t outMin = Q15_MIN, outMax = Q15_MAX;
  q31_t integ = 0;                   ///< ★ 积分器用 Q31（位宽 = Q15 + 16 位小数）
  bool  antiWindup = true;

  void reset() { integ = 0; }

  /**
   * @param err    Q15 误差
   * @param dt_us  控制周期（微秒，整数）
   * ★ 为什么 dt 不用 Q15 表示：1/5000 s 在 Q15 里只有 6~7，量化误差 15%！
   *   用微秒整数既精确又省事（这也是定点化的一个实用经验）。
   */
  q15_t update(q15_t err, uint32_t dt_us) {
    const q15_t p = mul(kp, err);
    // 积分增量，Q31：inc = ki·err·dt << 16
    //   mul(ki,err) 是 Q15 → 乘 dt_us/1e6 得到 Q15 增量 → 左移 16 位升到 Q31
    //   ★ 若只用 Q15 存积分，小信号时增量会被截断成 0（积分器"爬不动"）——
    //     这正是 E7.1 说的"PI 积分器位宽不足"问题，扩到 Q31 即可解决。
    const int64_t inc = (((int64_t)mul(ki, err) << 16) * (int64_t)dt_us) / 1000000LL;

    q31_t unsat = (q31_t)p + (q31_t)(integ >> 16);
    q31_t sat = unsat;
    if (sat > outMax) sat = outMax;
    if (sat < outMin) sat = outMin;

    if (antiWindup) {
      const bool pushUp   = (unsat > outMax) && (err > 0);
      const bool pushDown = (unsat < outMin) && (err < 0);
      if (!(pushUp || pushDown)) integ = sat31((int64_t)integ + inc);
    } else {
      integ = sat31((int64_t)integ + inc);
    }
    // 积分器限幅（±2 倍输出范围，够用且不会长期饱和回不来）
    int64_t iMax = (int64_t)outMax * 2 * 65536;
    int64_t iMin = (int64_t)outMin * 2 * 65536;
    if (iMax >  2147483647LL) iMax =  2147483647LL;
    if (iMin < -2147483647LL) iMin = -2147483647LL;
    if ((int64_t)integ > iMax) integ = (q31_t)iMax;
    if ((int64_t)integ < iMin) integ = (q31_t)iMin;

    return sat16(sat);
  }
};

}  // namespace focq15
