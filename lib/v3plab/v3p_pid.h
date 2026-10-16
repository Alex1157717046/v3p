/**
 * @file v3p_pid.h
 * @brief PI / PID 控制器（带三种抗饱和方案，供 E3.1 / E3.5 对比）
 *
 * 抗饱和方案：
 *   None     —— 什么都不做（用来复现积分饱和的超调）
 *   Clamp    —— 条件积分：输出饱和且误差继续推向饱和方向时停止积分
 *   BackCalc —— 反计算：把 (sat - unsat) 通过 kaw 反馈回积分器
 */
#pragma once

#include <math.h>

namespace v3p {

enum class AntiWindup { None, Clamp, BackCalc };

class PiController {
 public:
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;

  float outMin = -1e30f;
  float outMax =  1e30f;
  float intMin = -1e30f;
  float intMax =  1e30f;

  AntiWindup aw = AntiWindup::Clamp;
  float kaw = 1.0f;          // 反计算增益（BackCalc 用）

  void reset(float integ = 0.0f) {
    integ_ = integ;
    out_   = 0.0f;
    dPrev_ = 0.0f;
    first_ = true;
  }

  /// 标准更新（Clamp / None 模式）
  float update(float err, float dt) {
    const float p = kp * err;

    float d = 0.0f;
    if (kd > 0.0f && dt > 0.0f) {
      if (first_) { dPrev_ = err; first_ = false; }
      d = kd * (err - dPrev_) / dt;
      dPrev_ = err;
    }

    const float unsat = p + integ_ + d;
    const float sat   = clampf(unsat, outMin, outMax);

    // ---- 积分更新 ----
    switch (aw) {
      case AntiWindup::None:
        integ_ += ki * err * dt;
        break;
      case AntiWindup::Clamp: {
        const bool pushingUp   = (unsat > outMax) && (err > 0.0f);
        const bool pushingDown = (unsat < outMin) && (err < 0.0f);
        if (!(pushingUp || pushingDown)) integ_ += ki * err * dt;
        break;
      }
      case AntiWindup::BackCalc:
        integ_ += (ki * err + kaw * (sat - unsat)) * dt;
        break;
    }
    integ_ = clampf(integ_, intMin, intMax);
    out_ = sat;
    return out_;
  }

  /// 变体：外部已知实际施加值（如占空比限幅后的电压），用于更准的反计算
  float update(float err, float dt, float applied) {
    update(err, dt);
    if (aw == AntiWindup::BackCalc) {
      integ_ += kaw * (applied - out_) * dt;
      integ_ = clampf(integ_, intMin, intMax);
    }
    return out_;
  }

  float integral() const { return integ_; }
  float output()   const { return out_; }
  void  setIntegral(float v) { integ_ = clampf(v, intMin, intMax); }

 private:
  static float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  }

  float integ_ = 0.0f;
  float out_   = 0.0f;
  float dPrev_ = 0.0f;
  bool  first_ = true;
};

/// 一阶低通（用于 E3.6 滤波器与相位代价实验）
class LowPass {
 public:
  /// @param fc_hz 截止频率，0 = 直通
  void setCutoff(float fc_hz, float sample_hz) {
    if (fc_hz <= 0.0f) { alpha_ = 1.0f; return; }
    const float rc = 1.0f / (2.0f * 3.14159265f * fc_hz);
    const float dt = 1.0f / sample_hz;
    alpha_ = dt / (rc + dt);
  }
  void reset(float v = 0.0f) { y_ = v; }
  float update(float x) { y_ += alpha_ * (x - y_); return y_; }
  float value() const { return y_; }
  /// 该滤波器在当前采样率下的相位滞后（度），供 E3.6 画理论曲线
  static float phaseLagDeg(float fc_hz, float f_hz) {
    return -atanf(f_hz / fc_hz) * 180.0f / 3.14159265f;
  }
 private:
  float alpha_ = 1.0f;
  float y_ = 0.0f;
};

/// 二阶陷波滤波器（E5.6 机械谐振抑制）
class NotchFilter {
 public:
  /**
   * @param f0_hz  陷波中心频率
   * @param bw_hz  带宽（-3dB 宽度），越小越"窄"、相位代价越小
   * @param fs_hz  采样率
   */
  void design(float f0_hz, float bw_hz, float fs_hz) {
    if (f0_hz <= 0.0f || f0_hz >= fs_hz * 0.5f || bw_hz <= 0.0f) { enabled_ = false; return; }
    const float w0 = 2.0f * 3.14159265f * f0_hz / fs_hz;
    const float q  = f0_hz / bw_hz;
    const float alpha = sinf(w0) / (2.0f * q);
    const float cw = cosf(w0);
    const float a0 = 1.0f + alpha;
    b0_ = 1.0f / a0;
    b1_ = -2.0f * cw / a0;
    b2_ = 1.0f / a0;
    a1_ = -2.0f * cw / a0;
    a2_ = (1.0f - alpha) / a0;
    enabled_ = true;
  }
  void disable() { enabled_ = false; }
  bool enabled() const { return enabled_; }
  void reset() { x1_ = x2_ = y1_ = y2_ = 0.0f; }
  float update(float x) {
    if (!enabled_) return x;
    const float y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_; x1_ = x;
    y2_ = y1_; y1_ = y;
    return y;
  }
 private:
  bool  enabled_ = false;
  float b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
  float x1_ = 0, x2_ = 0, y1_ = 0, y2_ = 0;
};

}  // namespace v3p
