/**
 * @file v3p_math.h
 * @brief FOC 数学：Clarke / Park / 反Park / SPWM / SVPWM(七段&五段) / 六步 / 死区补偿
 *
 * 约定：
 *   - Clarke/Park 采用【幅值不变】变换：三相正弦幅值 I → αβ 矢量长度 I。
 *   - d 轴对齐转子磁链，q 轴超前 d 轴 90° 电角度。
 *   - duty ∈ [0,1]，1 = 上管常通。
 *     ⚠ V3P 板上 FD6287T 的 HIN 与 LIN# 并联，duty=1 会让自举电容持续放电（见 E1.7）。
 *   - 线性区极限：SPWM 相电压峰值 = Vdc/2；SVPWM = Vdc/√3（高 15.47%）。
 */
#pragma once

#include <math.h>
#include <stdint.h>

namespace v3p {

constexpr float PI_F     = 3.14159265358979f;
constexpr float TWO_PI_F = 6.28318530717959f;
constexpr float SQRT3_F  = 1.73205080756888f;
constexpr float ONE_BY_SQRT3_F = 0.57735026918963f;

// ============================ 角度工具 ============================

inline float wrapPi(float a) {
  while (a > PI_F) a -= TWO_PI_F;
  while (a <= -PI_F) a += TWO_PI_F;
  return a;
}

inline float wrap2Pi(float a) {
  while (a >= TWO_PI_F) a -= TWO_PI_F;
  while (a < 0.0f) a += TWO_PI_F;
  return a;
}

inline float angleDiff(float a, float b) { return wrapPi(a - b); }

// ============================ 数据结构 ============================

struct ABC { float a, b, c; };
struct AlphaBeta { float alpha, beta; };
struct DQ { float d, q; };
struct Duty { float a, b, c; };

// ============================ Clarke / Park ============================

/**
 * Clarke（两电流传感器版本）：
 *   iα = ia
 *   iβ = (ia + 2·ib) / √3
 * 验证：ia=I·cosθ, ib=I·cos(θ−120°) → (iα,iβ) = I·(cosθ, sinθ)
 */
inline AlphaBeta clarke(float ia, float ib) {
  AlphaBeta o;
  o.alpha = ia;
  o.beta  = (ia + 2.0f * ib) * ONE_BY_SQRT3_F;
  return o;
}

/// 三传感器版本（E2.3 里与两传感器版本对比，用于验证 KCL 假设）
inline AlphaBeta clarke3(float ia, float ib, float ic) {
  AlphaBeta o;
  o.alpha = ia;
  o.beta  = (ib - ic) * ONE_BY_SQRT3_F;   // 不用 KCL 的等价形式
  return o;
}

inline DQ park(const AlphaBeta& in, float theta_e) {
  const float s = sinf(theta_e), c = cosf(theta_e);
  DQ o;
  o.d =  in.alpha * c + in.beta * s;
  o.q = -in.alpha * s + in.beta * c;
  return o;
}

inline AlphaBeta invPark(const DQ& in, float theta_e) {
  const float s = sinf(theta_e), c = cosf(theta_e);
  AlphaBeta o;
  o.alpha = in.d * c - in.q * s;
  o.beta  = in.d * s + in.q * c;
  return o;
}

/// 三相电压参考（幅值 = 相电压峰值）
inline ABC abToAbc(const AlphaBeta& v) {
  ABC o;
  o.a = v.alpha;
  o.b = -0.5f * v.alpha + 0.5f * SQRT3_F * v.beta;
  o.c = -0.5f * v.alpha - 0.5f * SQRT3_F * v.beta;
  return o;
}

// ============================ 调制 ============================

/// SPWM：duty_x = 0.5 + v_x/Vdc
inline Duty spwm(const AlphaBeta& v, float vdc) {
  const float k = 1.0f / vdc;
  const ABC p = abToAbc(v);
  Duty d;
  d.a = 0.5f + p.a * k;
  d.b = 0.5f + p.b * k;
  d.c = 0.5f + p.c * k;
  return d;
}

/**
 * SVPWM 七段式（显式扇区法）—— E1.3 核心
 *
 * 推导（幅值不变约定下，六个有效矢量长度为 2Vdc/3）：
 *   令 uα=vα/Vdc, uβ=vβ/Vdc
 *   X = √3·uβ,  Y = (3uα + √3·uβ)/2,  Z = (−3uα + √3·uβ)/2
 *   扇区 1..6 的相邻矢量作用时间：
 *     1: T1=−Z, T2= X      2: T1= Y, T2= Z      3: T1= X, T2=−Y
 *     4: T1= Z, T2=−X      5: T1=−Y, T2=−Z      6: T1=−X, T2= Y
 *   t0 = (1 − T1 − T2)/2，序列 000→U1→U2→111→U2→U1→000
 *
 * @param sector_out 可选，返回扇区号 1~6（0 表示零矢量附近）
 * @param m_out      可选，返回调制度 m = |v|/(Vdc/√3)（m=1 为线性区极限）
 */
inline Duty svpwm7(const AlphaBeta& v, float vdc, int* sector_out = nullptr,
                   float* m_out = nullptr) {
  const float inv_vdc = 1.0f / vdc;
  const float ua = v.alpha * inv_vdc;
  const float ub = v.beta  * inv_vdc;

  if (m_out) *m_out = sqrtf(ua * ua + ub * ub) * SQRT3_F;

  // ---- 扇区判断（比较法）----
  const float A = ub;
  const float B = 0.5f * (SQRT3_F * ua - ub);
  const float C = 0.5f * (-SQRT3_F * ua - ub);
  const int N = (A > 0.0f ? 1 : 0) | (B > 0.0f ? 2 : 0) | (C > 0.0f ? 4 : 0);
  int sec = 0;
  switch (N) {
    case 3: sec = 1; break;
    case 1: sec = 2; break;
    case 5: sec = 3; break;
    case 4: sec = 4; break;
    case 6: sec = 5; break;
    case 2: sec = 6; break;
    default: sec = 0; break;   // 零矢量
  }
  if (sector_out) *sector_out = sec;
  if (sec == 0) { Duty z{0.5f, 0.5f, 0.5f}; return z; }

  // ---- 作用时间 ----
  const float X = SQRT3_F * ub;
  const float Y = 0.5f * (3.0f * ua + SQRT3_F * ub);
  const float Z = 0.5f * (-3.0f * ua + SQRT3_F * ub);

  float T1 = 0.0f, T2 = 0.0f;
  switch (sec) {
    case 1: T1 = -Z; T2 =  X; break;
    case 2: T1 =  Y; T2 =  Z; break;
    case 3: T1 =  X; T2 = -Y; break;
    case 4: T1 =  Z; T2 = -X; break;
    case 5: T1 = -Y; T2 = -Z; break;
    default:T1 = -X; T2 =  Y; break;
  }

  // ---- 过调制：T1+T2 > 1 时等比缩放 ----
  const float Tsum = T1 + T2;
  if (Tsum > 1.0f) {
    const float k = 1.0f / Tsum;
    T1 *= k;
    T2 *= k;
  }
  const float t0 = 0.5f * (1.0f - T1 - T2);

  Duty d{0, 0, 0};
  switch (sec) {
    case 1: d.a = t0 + T1 + T2; d.b = t0 + T2;      d.c = t0; break;
    case 2: d.a = t0 + T1;      d.b = t0 + T1 + T2; d.c = t0; break;
    case 3: d.a = t0;           d.b = t0 + T1 + T2; d.c = t0 + T2; break;
    case 4: d.a = t0;           d.b = t0 + T1;      d.c = t0 + T1 + T2; break;
    case 5: d.a = t0 + T2;      d.b = t0;           d.c = t0 + T1 + T2; break;
    default:d.a = t0 + T1 + T2; d.b = t0;           d.c = t0 + T1; break;
  }
  return d;
}

/**
 * 载波比较式 SVPWM（min-max 零序注入）—— 与 svpwm7 在线性区应完全等价，
 * E1.3 用它做交叉验证；也是过调制/DPWM 的自然实现方式。
 */
inline Duty svpwmMinMax(const AlphaBeta& v, float vdc) {
  const ABC p = abToAbc(v);
  const float vmax = fmaxf(p.a, fmaxf(p.b, p.c));
  const float vmin = fminf(p.a, fminf(p.b, p.c));
  const float off  = -0.5f * (vmax + vmin);   // 零序注入
  const float k    = 1.0f / vdc;
  Duty d;
  d.a = 0.5f + (p.a + off) * k;
  d.b = 0.5f + (p.b + off) * k;
  d.c = 0.5f + (p.c + off) * k;
  return d;
}

/**
 * 五段式 / DPWM：把七段式两个零矢量合并到一端（等于刨掉公共的 t0）
 * → 每周期开关次数 6→4，代价是纹波增大（E1.4 实测）
 * 减去公共分量不改变线电压，只改变共模。
 */
inline Duty svpwm5(const AlphaBeta& v, float vdc) {
  const Duty d7 = svpwm7(v, vdc, nullptr, nullptr);
  const float mn = fminf(d7.a, fminf(d7.b, d7.c));
  Duty d;
  d.a = d7.a - mn;
  d.b = d7.b - mn;
  d.c = d7.c - mn;
  return d;
}

/**
 * 六步换相（E1.1）：1=上管常通，0=下管常通，0.5=该相斩波（平均 0V）
 * 顺序：A+B− / A+C− / B+C− / B+A− / C+A− / C+B−
 */
inline void sixStep(int step6, float* da, float* db, float* dc) {
  static const float tbl[6][3] = {
      {1.0f, 0.0f, 0.5f},
      {1.0f, 0.5f, 0.0f},
      {0.5f, 1.0f, 0.0f},
      {0.0f, 1.0f, 0.5f},
      {0.0f, 0.5f, 1.0f},
      {0.5f, 0.0f, 1.0f},
  };
  const int s = ((step6 % 6) + 6) % 6;
  *da = tbl[s][0];
  *db = tbl[s][1];
  *dc = tbl[s][2];
}

/**
 * 死区电压补偿：ΔV = sign(i)·(Td/Ts)·Vdc
 * 用 tanh 在过零附近平滑（硬 sign 会引入新的过零畸变）。
 * 注意：V3P 板的死区由 FD6287T 内部固定，软件只能【补偿】，不能【设置】。
 */
inline float deadTimeComp(float i, float i_eps, float td, float ts, float vdc) {
  return tanhf(i / i_eps) * (td / ts) * vdc;
}

/// 占空比钳位（防止超出 [0,1] 丢脉冲）
inline void clampDuty(Duty& d, float dmin = 0.0f, float dmax = 1.0f) {
  d.a = fmaxf(dmin, fminf(dmax, d.a));
  d.b = fmaxf(dmin, fminf(dmax, d.b));
  d.c = fmaxf(dmin, fminf(dmax, d.c));
}

}  // namespace v3p
