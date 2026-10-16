/**
 * @file v3p_pwm.h
 * @brief 三相 PWM 驱动（ESP32 LEDC）
 *
 * ★ 为什么用 LEDC 而不是 MCPWM：
 *   V3P 板每相只有 1 根 PWM 线（FD6287T 的 HIN 与 LIN# 在板上并联），
 *   所以互补输出 + 硬件死区寄存器在本板上【没有意义】——官方 DengFOC 也用 LEDC。
 *   本驱动暴露"动态改频率"能力，用于 E1.7（低载波比）与 E1.4（纹波）实验。
 */
#pragma once

#include <stdint.h>
#include "v3p_math.h"

namespace v3p {

class PwmDriver {
 public:
  /**
   * @param chBase LEDC 通道基址（M0 用 0，M1 用 3）
   */
  void begin(int pinA, int pinB, int pinC, int enPin, int chBase = 0);

  /// 动态设置 PWM 频率（Hz）。会自动选择满足该频率的最大分辨率（8~14 bit）。
  bool setFrequency(float fsw);

  /// 设置三相占空比，输入会钳位到 [0,1]
  void setDuty(const Duty& d);

  /// 直接写 0~1 的三路占空比
  void setDuty(float a, float b, float c);

  /// 使能/失能驱动板（MOTOR_EN）
  void enable(bool on);

  /// 三相下管全开（短路制动，电流会很大，慎用）
  void brake();

  /// 关断：占空比全 0 + 失能
  void coast();

  float    frequency() const { return fsw_; }
  uint32_t maxDuty()  const { return maxDuty_; }
  bool     ready()    const { return inited_; }

 private:
  void apply_();

  int       pins_[3] = {0, 0, 0};
  int       ch_[3]   = {0, 1, 2};
  int       en_      = -1;
  bool      inited_  = false;
  bool      enabled_ = false;
  float     fsw_     = 20000.0f;
  uint32_t  resBits_ = 11;
  uint32_t  maxDuty_ = 2047;
  Duty      duty_{0.0f, 0.0f, 0.0f};
};

}  // namespace v3p
