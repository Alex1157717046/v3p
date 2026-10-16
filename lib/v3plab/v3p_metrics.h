/**
 * @file v3p_metrics.h
 * @brief 阶跃响应指标（E3.1/E3.3/E3.5 的"量化验收"用）
 *
 * 输出：上升时间(10%→90%)、超调量(%)、调节时间(±2% 带)、稳态误差
 * 另给出由上升时间估算的闭环带宽：BW ≈ 0.35 / tr（一阶近似）
 */
#pragma once

#include <math.h>
#include <stdint.h>

namespace v3p {

class StepMetrics {
 public:
  /// @param ref     阶跃后的目标值
  /// @param initial 阶跃前的输出值
  /// @param t0      阶跃发生的时刻（秒）
  void start(float ref, float initial, float t0) {
    ref_ = ref;
    init_ = initial;
    t0_ = t0;
    done_ = false;
    tr_ = ts_ = -1.0f;
    ymax_ = -1e30f;
    ymin_ =  1e30f;
    cross10_ = -1.0f;
    cross90_ = -1.0f;
    tLastIn_ = -1.0f;
    yEnd_ = initial;
    haveT0_ = true;
  }

  /// 每个控制周期喂一次
  void feed(float y, float t) {
    if (!haveT0_) return;
    const float span = ref_ - init_;
    const float dir  = (span >= 0.0f) ? 1.0f : -1.0f;
    const float y10  = init_ + 0.1f * span;
    const float y90  = init_ + 0.9f * span;

    yEnd_ = y;
    const float d = dir * (y - init_) / fabsf(span);
    if (d > ymax_) ymax_ = d;
    if (d < ymin_) ymin_ = d;

    if (cross10_ < 0.0f && ((dir > 0 && y >= y10) || (dir < 0 && y <= y10))) cross10_ = t;
    if (cross90_ < 0.0f && ((dir > 0 && y >= y90) || (dir < 0 && y <= y90))) cross90_ = t;
    if (cross10_ >= 0.0f && cross90_ >= 0.0f && tr_ < 0.0f) tr_ = cross90_ - cross10_;

    // ±2% 带
    const float band = 0.02f * fabsf(span);
    if (fabsf(y - ref_) <= band) tLastIn_ = t;
    // 阶跃后 5×tr（或 200 ms）后判定结束
    if (tr_ > 0.0f && (t - t0_) > fmaxf(5.0f * tr_, 0.05f)) done_ = true;
    if ((t - t0_) > 2.0f) done_ = true;
  }

  bool  done() const { return done_; }
  float riseTime()   const { return tr_; }
  float settlingTime() const { return (tLastIn_ > 0.0f) ? (tLastIn_ - t0_) : -1.0f; }
  float overshootPct() const {
    if (!haveT0_) return 0.0f;
    const float over = (ymax_ > 1.0f) ? (ymax_ - 1.0f) : 0.0f;
    return over * 100.0f;
  }
  float steadyError() const { return ref_ - yEnd_; }
  /// 由上升时间估算的闭环带宽（Hz，一阶近似）
  float bandwidthHz() const { return (tr_ > 1e-6f) ? (0.35f / tr_) : -1.0f; }
  float ref() const { return ref_; }

 private:
  float ref_ = 0, init_ = 0, t0_ = 0, yEnd_ = 0;
  float tr_ = -1, ts_ = -1;
  float ymax_ = -1e30f, ymin_ = 1e30f;
  float cross10_ = -1, cross90_ = -1, tLastIn_ = -1;
  bool  done_ = false, haveT0_ = false;
};

}  // namespace v3p
