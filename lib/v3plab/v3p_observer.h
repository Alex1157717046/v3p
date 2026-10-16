/**
 * @file v3p_observer.h
 * @brief 转子位置观测器：磁链观测器 / 滑模观测器 / PLL（阶段 4 的核心）
 *
 * ★ 本课程做观测器的独特优势：板上装了 AS5600，可以给出【真值】。
 *   控制仍然用编码器，观测器【并行运行但不参与控制】，
 *   于是可以量化"观测器角度误差 vs 转速"这条曲线 —— 见 12.3。
 *
 * 约定（与 v3p_math.h 一致，幅值不变 Clarke）：
 *   磁链   λ = λm·[cosθe, sinθe]        （表贴式、id=0）
 *   反 EMF e = ωe·λm·[−sinθe, cosθe]   （注意是超前 θe 90° 的旋转矢量）
 *   因此  θe = atan2(λβ, λα) 且 θe = atan2(−eα, eβ)
 */
#pragma once

#include "v3p_math.h"

namespace v3p {

enum class ObserverType {
  FluxPure,   ///< 纯积分（会漂移，用来复现问题）
  FluxLpf,    ///< 可编程低通磁链观测器（ωc 跟随转速）★ 推荐
  Smo,        ///< 滑模观测器
};

enum class SmoFn { Sign, Sigmoid, Sat };

class Observer {
 public:
  // ---- 参数（由 E5.1/E5.2/E4.1 实测替换）----
  float rs     = 10.0f;
  float ls     = 0.010f;
  float lambda = 0.0079f;

  // ---- 磁链观测器 ----
  ObserverType type = ObserverType::FluxLpf;
  float k      = 0.2f;      ///< ωc = k·|ωe|（相位滞后 = atan(1/k)，常数！）
  float wc0    = 5.0f;      ///< 最低截止（rad/s），防止零速时 ωc=0
  bool  phaseComp = true;   ///< 是否加常数相位补偿 atan(1/k)
  bool  subtractLi = true;  ///< 是否先减去 L·i 再取角度（更准）

  // ---- 滑模观测器 ----
  SmoFn smoFn = SmoFn::Sigmoid;
  float smoK  = 20.0f;      ///< 滑模增益（越大收敛越快、抖振越大）
  float smoA  = 0.05f;      ///< sigmoid/sat 的边界层宽度
  float smoLpfFc = 500.0f;  ///< 提取反 EMF 的低通截止（Hz）

  // ---- PLL ----
  bool  usePll = true;
  float pllKp = 0.0f;
  float pllKi = 0.0f;
  /** 按自然频率 ωn 与阻尼 ζ 设计 PLL：Kp=2ζωn, Ki=ωn² */
  void designPll(float wn_rad_s, float zeta) {
    pllKp = 2.0f * zeta * wn_rad_s;
    pllKi = wn_rad_s * wn_rad_s;
  }

  void reset() {
    fluxA_ = fluxB_ = 0.0f;
    iHatA_ = iHatB_ = 0.0f;
    eA_ = eB_ = 0.0f;
    theta_ = 0.0f;
    omega_ = 0.0f;
    pllInt_ = 0.0f;
    raw_ = 0.0f;
  }

  /** 预置 PLL 角度（E4.6 进阶：VESC 的"预置观测器磁链"做法） */
  void presetAngle(float th) {
    theta_ = wrapPi(th);
    pllInt_ = 0.0f;
  }

  /**
   * 主更新
   * @param vab   实际施加的 αβ 电压（取 FocMotor::vAlphaBeta）
   * @param iab   实测 αβ 电流（取 FocMotor::iAlphaBeta）
   * @param dt    周期（s）
   * @param omegaRef 参考电角速度（rad/s，用编码器速度或估计速度都行；
   *                 磁链观测器的 ωc 需要它）
   */
  void update(const AlphaBeta& vab, const AlphaBeta& iab, float dt, float omegaRef) {
    switch (type) {
      case ObserverType::Smo:      updateSmo_(vab, iab, dt); break;
      case ObserverType::FluxPure: updateFlux_(vab, iab, dt, 0.0f, false); break;
      case ObserverType::FluxLpf:
      default:                     updateFlux_(vab, iab, dt, omegaRef, true); break;
    }

    // 得到反 EMF 或磁链后，提取角度（atan2 直算 或 PLL）
    float th = 0.0f;
    float eA = 0.0f, eB = 0.0f;
    if (type == ObserverType::Smo) {
      eA = eA_; eB = eB_;
      th = atan2f(-eA_, eB_);
    } else {
      // 磁链 → 反 EMF（对时间微分，或用 λ×ω 关系）
      const float dA = fluxA_ - lastFluxA_;
      const float dB = fluxB_ - lastFluxB_;
      lastFluxA_ = fluxA_; lastFluxB_ = fluxB_;
      if (dt > 1e-6f) { eA = dA / dt; eB = dB / dt; }
      float lA = fluxA_, lB = fluxB_;
      if (subtractLi) { lA -= ls * iab.alpha; lB -= ls * iab.beta; }
      th = atan2f(lB, lA);
      if (phaseComp && type == ObserverType::FluxLpf && k > 1e-3f) {
        th += atanf(1.0f / k);      // ★ 常数相位超前（关键技巧）
      }
    }
    raw_ = wrapPi(th);

    if (usePll) {
      // 角度误差 ε = −(eα·cosθ̂ + eβ·sinθ̂)/|e|
      const float emag = sqrtf(eA * eA + eB * eB) + 1e-6f;
      const float eps = -(eA * cosf(theta_) + eB * sinf(theta_)) / emag;
      pllInt_ += pllKi * eps * dt;
      omega_ = pllKp * eps + pllInt_;
      theta_ = wrapPi(theta_ + omega_ * dt);
    } else {
      theta_ = raw_;
      omega_ = (dt > 1e-6f) ? wrapPi(theta_ - thetaPrev_) / dt : 0.0f;
      thetaPrev_ = theta_;
    }
  }

  // ---- 输出 ----
  float theta() const { return theta_; }      ///< 估计电角度（rad，归一化）
  float omega() const { return omega_; }      ///< 估计电角速度（rad/s）
  float rawAngle() const { return raw_; }     ///< atan2 直接算的角度（未过 PLL）
  float fluxAlpha() const { return fluxA_; }
  float fluxBeta()  const { return fluxB_; }
  float fluxMag()   const { return sqrtf(fluxA_ * fluxA_ + fluxB_ * fluxB_); }  ///< ≈ λm
  float bemfMag()   const { return sqrtf(eA_ * eA_ + eB_ * eB_); }
  float eAlpha() const { return eA_; }
  float eBeta()  const { return eB_; }
  float pllErr() const { return pllInt_; }

 private:
  void updateFlux_(const AlphaBeta& v, const AlphaBeta& i, float dt, float omegaRef, bool lpf) {
    const float errA = v.alpha - rs * i.alpha;
    const float errB = v.beta  - rs * i.beta;
    if (lpf) {
      const float wc = k * fabsf(omegaRef) + wc0;
      fluxA_ += (errA - wc * fluxA_) * dt;
      fluxB_ += (errB - wc * fluxB_) * dt;
    } else {
      fluxA_ += errA * dt;      // 纯积分 → 会漂移（就是要给你看）
      fluxB_ += errB * dt;
    }
  }

  void updateSmo_(const AlphaBeta& v, const AlphaBeta& i, float dt) {
    // 电流滑模观测器：dî/dt = (v − R·i − z)/L ，z = K·f(î − i)
    const float ea = iHatA_ - i.alpha;
    const float eb = iHatB_ - i.beta;
    float za = 0.0f, zb = 0.0f;
    switch (smoFn) {
      case SmoFn::Sign:    za = smoK * ((ea > 0) ? 1.0f : -1.0f);
                           zb = smoK * ((eb > 0) ? 1.0f : -1.0f); break;
      case SmoFn::Sat:     za = smoK * fmaxf(-1.0f, fminf(1.0f, ea / smoA));
                           zb = smoK * fmaxf(-1.0f, fminf(1.0f, eb / smoA)); break;
      case SmoFn::Sigmoid:
      default:             za = smoK * tanhf(ea / smoA);
                           zb = smoK * tanhf(eb / smoA); break;
    }
    const float invL = 1.0f / fmaxf(ls, 1e-6f);
    iHatA_ += (v.alpha - rs * i.alpha - za) * invL * dt;
    iHatB_ += (v.beta  - rs * i.beta  - zb) * invL * dt;

    // z 里含有反 EMF 信息，低通提取（z 的等效切换项即反 EMF 估计）
    const float rc = 1.0f / (TWO_PI_F * fmaxf(smoLpfFc, 1.0f));
    const float al = dt / (rc + dt);
    eA_ += al * (za - eA_);
    eB_ += al * (zb - eB_);

    fluxA_ = eA_;   // 便于统一读取
    fluxB_ = eB_;
  }

  float fluxA_ = 0, fluxB_ = 0;
  float lastFluxA_ = 0, lastFluxB_ = 0;
  float iHatA_ = 0, iHatB_ = 0;
  float eA_ = 0, eB_ = 0;
  float theta_ = 0, omega_ = 0, raw_ = 0, thetaPrev_ = 0;
  float pllInt_ = 0;
};

}  // namespace v3p
