#include "v3p_encoder.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

#include "v3p_math.h"

namespace v3p {

bool As5600::begin(TwoWire* wire, int sda, int scl, uint32_t hz, int dir) {
  wire_ = wire;
  dir_  = (dir >= 0) ? 1 : -1;
  wire_->begin(sda, scl, hz);

  const uint16_t a = readReg16_(REG_RAWANG);
  ok_ = (a <= 4095);

  lastRaw_ = raw_ = a;
  zeroRaw_ = (float)a;
  angle_   = 0.0f;
  turns_   = 0;
  lastUs_  = micros();
  return ok_;
}

uint8_t As5600::readReg8_(uint8_t reg) {
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return 0;
  if (wire_->requestFrom((int)I2C_ADDR, 1) != 1) return 0;
  return (uint8_t)wire_->read();
}

uint16_t As5600::readReg16_(uint8_t reg) {
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return 0;
  if (wire_->requestFrom((int)I2C_ADDR, 2) != 2) return 0;
  const uint8_t hi = (uint8_t)wire_->read();
  const uint8_t lo = (uint8_t)wire_->read();
  return (uint16_t)((((uint16_t)hi << 8) | lo) & 0x0FFF);
}

bool As5600::update() {
  const uint32_t now = micros();
  const uint16_t r = readReg16_(REG_RAWANG);
  if (r > 4095) return false;

  // ---- 多圈计数：处理 0 ↔ 4095 回绕 ----
  int32_t d = (int32_t)r - (int32_t)lastRaw_;
  if (d > 2048)       { turns_--; }
  else if (d < -2048) { turns_++; }

  const float dRad = dir_ * (float)d * (TWO_PI_F / 4096.0f);
  lastRaw_ = raw_ = r;
  angle_  += dRad;

  // ---- 速度：差分 + 一阶低通 ----
  const float dt = (float)(now - lastUs_) * 1e-6f;
  lastUs_ = now;
  if (dt > 1e-6f && dt < 0.1f) {
    velRaw_ = dRad / dt;
    const float rc    = (velFc_ > 1.0f) ? (1.0f / (TWO_PI_F * velFc_)) : 0.0f;
    const float alpha = (rc > 0.0f) ? (dt / (rc + dt)) : 1.0f;
    velocity_ += alpha * (velRaw_ - velocity_);
  }
  return true;
}

void As5600::setZero() {
  zeroRaw_ = (float)raw_ - (float)turns_ * 4096.0f;
  turns_    = 0;
  angle_    = 0.0f;
  velocity_ = 0.0f;
}

void As5600::setVelocityLpf(float fc_hz, float sample_hz) {
  velFc_ = fc_hz;
  (void)sample_hz;
}

bool As5600::magnetDetected() { return (readReg8_(REG_STATUS) & 0x20) != 0; }
bool As5600::magnetWeak()     { return (readReg8_(REG_STATUS) & 0x10) != 0; }
bool As5600::magnetStrong()   { return (readReg8_(REG_STATUS) & 0x08) != 0; }
uint8_t As5600::agc()         { return readReg8_(REG_AGC); }

}  // namespace v3p
