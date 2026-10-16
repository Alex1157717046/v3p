/**
 * @file v3p_current.h
 * @brief 相电流采样：INA240A2 ×2/路 + 0.01 Ω inline 分流
 *
 * 拓扑：分流电阻串在【相线】上（inline），不是低边采样。
 *   → 采样窗口不受下管导通时间限制，没有"高调制比盲区"问题，
 *     但共模电压高（跟随相电压跳变）。
 * 量程：±(1.65 V)/(0.01 Ω × 50) = ±3.3 A
 *   → 对 2208 云台电机（堵转 1.4 A）够用；换 1 mΩ 可到 ±33 A。
 */
#pragma once

#include <stdint.h>

namespace v3p {

class CurrentSense {
 public:
  /**
   * @param pin1   第一路采样引脚（如 M0_OUT1_CS = 39）
   * @param pin2   第二路采样引脚（如 M0_OUT2_CS = 36）
   * @param v_per_a  灵敏度 [V/A]，默认 0.01 Ω × 50 = 0.5
   * @param vref     零点电压（理想 1.65 V = Vdd/2）
   */
  void begin(int pin1, int pin2, float v_per_a = 0.5f, float vref = 1.65f);

  /// 零偏标定：必须在【电机不通电、无电流】时调用
  void calibrate(int samples = 512);

  /// 读三相电流（第三相由 KCL 推出）
  void read(float& ia, float& ib, float& ic);

  /// 只读两路原始电流
  void readTwo(float& i1, float& i2);

  /// 原始 ADC 码值 / 电压（供 E2.2、E2.4 使用）
  void readRaw(uint16_t& r1, uint16_t& r2) const;
  void readVolts(float& v1, float& v2) const;

  /// 每路增益方向（板上 INA240 输出方向可能相反，做实验时用 ±1 标定）
  float sign1 = 1.0f, sign2 = 1.0f;

  float vPerAmp() const { return v_per_a_; }
  float offset1() const { return offset1_; }
  float offset2() const { return offset2_; }

  /// 把 ADC 码值换成电压（线性，供快速路径复用）
  float rawToVolt(uint16_t raw) const {
    return (float)raw * ADC_FS_V_ / 4095.0f;
  }

 private:
  int   pin1_ = -1, pin2_ = -1;
  float v_per_a_ = 0.5f;
  float vref_    = 1.65f;
  float offset1_ = 0.0f;   // 电压域零偏
  float offset2_ = 0.0f;
  static constexpr float ADC_FS_V_ = 3.3f;
};

}  // namespace v3p
