#include "v3p_pwm.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

namespace v3p {

void PwmDriver::begin(int pinA, int pinB, int pinC, int enPin, int chBase) {
  pins_[0] = pinA; pins_[1] = pinB; pins_[2] = pinC;
  ch_[0] = chBase; ch_[1] = chBase + 1; ch_[2] = chBase + 2;
  en_ = enPin;

  if (en_ >= 0) {
    pinMode(en_, OUTPUT);
    digitalWrite(en_, LOW);   // 上电默认失能，最安全
  }
  setFrequency(fsw_);
  for (int i = 0; i < 3; i++) {
    ledcAttachPin(pins_[i], ch_[i]);
  }
  // 初始输出 0（下管导通、电机被短路制动）—— 但驱动板尚未使能，不会真的制动
  setDuty(0.0f, 0.0f, 0.0f);
  inited_ = true;
}

bool PwmDriver::setFrequency(float fsw) {
  if (fsw <= 0.0f) return false;
  // LEDC 时钟 80 MHz；bits = floor(log2(80e6/fsw))，钳到 [8,14]
  uint32_t bits = 0;
  float div = 80000000.0f / fsw;
  while (div >= 2.0f && bits < 14) { div *= 0.5f; bits++; }
  if (bits < 8)  bits = 8;
  if (bits > 14) bits = 14;

  resBits_ = bits;
  maxDuty_ = (1u << bits) - 1u;
  fsw_     = fsw;

  for (int i = 0; i < 3; i++) {
    ledcSetup(ch_[i], fsw, bits);
    // 运行时改频率后再 attach 一次，避免部分 Arduino-ESP32 版本丢绑定
    if (inited_) ledcAttachPin(pins_[i], ch_[i]);
  }
  if (inited_) apply_();
  return true;
}

void PwmDriver::setDuty(const Duty& d) { setDuty(d.a, d.b, d.c); }

void PwmDriver::setDuty(float a, float b, float c) {
  duty_.a = fmaxf(0.0f, fminf(1.0f, a));
  duty_.b = fmaxf(0.0f, fminf(1.0f, b));
  duty_.c = fmaxf(0.0f, fminf(1.0f, c));
  if (inited_ && enabled_) apply_();
}

void PwmDriver::apply_() {
  const float d[3] = {duty_.a, duty_.b, duty_.c};
  for (int i = 0; i < 3; i++) {
    ledcWrite(ch_[i], (uint32_t)(d[i] * (float)maxDuty_));
  }
}

void PwmDriver::enable(bool on) {
  enabled_ = on;
  if (on) {
    apply_();                       // 先建立占空比，再使能，避免上电冲击
    if (en_ >= 0) digitalWrite(en_, HIGH);
  } else {
    if (en_ >= 0) digitalWrite(en_, LOW);
  }
}

void PwmDriver::brake() {
  setDuty(0.0f, 0.0f, 0.0f);   // 三相下管全开
  if (inited_ && enabled_) apply_();
}

void PwmDriver::coast() {
  if (en_ >= 0) digitalWrite(en_, LOW);
  enabled_ = false;
  setDuty(0.0f, 0.0f, 0.0f);
}

}  // namespace v3p
