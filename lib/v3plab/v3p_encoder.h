/**
 * @file v3p_encoder.h
 * @brief AS5600 磁编码器（I2C）×2
 *
 * ⚠ 重要：AS5600 走 I2C，一次读取约 75~150 µs，【比 20 kHz 的 PWM 周期还长】。
 *   正确用法（也是本驱动的设计）：
 *     低优先级任务/定时器（1~2 kHz）里调 update() 做 I2C 读取
 *     电流环 ISR 只调用 angle() / velocity() 读缓存
 *   ——否则控制周期会被 I2C 拖爆（对应实验 E2.4、附录 G.3）。
 */
#pragma once

#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <Wire.h>
#endif

namespace v3p {

class As5600 {
 public:
  static constexpr uint8_t I2C_ADDR = 0x36;

  // 寄存器
  static constexpr uint8_t REG_STATUS  = 0x0B;
  static constexpr uint8_t REG_RAWANG  = 0x0C;
  static constexpr uint8_t REG_ANGLE   = 0x0E;
  static constexpr uint8_t REG_AGC     = 0x1A;
  static constexpr uint8_t REG_MAGNIT  = 0x1B;

  /**
   * @param wire  I2C 总线（M0 用 Wire，M1 用 Wire1）
   * @param sda   SDA_0=19 / SDA_1=23
   * @param scl   SCL_0=18 / SCL_1=5
   * @param dir   +1 / -1：机械正方向与计数方向的关系
   */
  bool begin(TwoWire* wire, int sda, int scl, uint32_t hz = 400000, int dir = 1);

  /// 读一次 I2C 并更新缓存（放在低优先级任务里调用）
  bool update();

  /// 缓存值（ISR 里安全）
  float    angle()    const { return angle_; }      // rad，连续多圈机械角
  float    velocity() const { return velocity_; }   // rad/s，低通后的微分
  float    velocityRaw() const { return velRaw_; }  // rad/s，未滤波
  uint16_t raw()      const { return raw_; }        // 0~4095
  uint32_t lastUpdateMicros() const { return lastUs_; }

  /// 把当前角度当作零点；把当前方向设成正方向
  void setZero();
  void setDirection(int dir) { dir_ = (dir >= 0) ? 1 : -1; }
  int  direction() const { return dir_; }

  /// 速度低通截止频率（Hz），0 = 不滤波
  void  setVelocityLpf(float fc_hz, float sample_hz);
  float velocityLpfCutoff() const { return velFc_; }

  /// 磁铁诊断（E2.1 用）
  bool  magnetDetected();
  bool  magnetWeak();
  bool  magnetStrong();
  uint8_t agc();

  bool ok() const { return ok_; }

 private:
  uint16_t readReg16_(uint8_t reg);
  uint8_t  readReg8_(uint8_t reg);

  TwoWire* wire_ = nullptr;
  bool     ok_   = false;
  int      dir_  = 1;

  uint16_t raw_      = 0;
  int32_t  turns_    = 0;      // 多圈计数
  uint16_t lastRaw_  = 0;
  float    zeroRaw_  = 0.0f;   // 零点（原始码值域）
  float    angle_    = 0.0f;   // rad
  float    velocity_ = 0.0f;
  float    velRaw_   = 0.0f;
  float    velFc_    = 200.0f;
  float    dtFilt_   = 0.0f;
  uint32_t lastUs_   = 0;
};

}  // namespace v3p
