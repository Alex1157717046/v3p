#include "v3p_current.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

namespace v3p {

void CurrentSense::begin(int pin1, int pin2, float v_per_a, float vref) {
  pin1_ = pin1;
  pin2_ = pin2;
  v_per_a_ = v_per_a;
  vref_ = vref;

  analogReadResolution(12);
  analogSetPinAttenuation(pin1_, ADC_11db);   // 0~3.3 V 满量程
  analogSetPinAttenuation(pin2_, ADC_11db);
  pinMode(pin1_, INPUT);
  pinMode(pin2_, INPUT);

  offset1_ = vref_;
  offset2_ = vref_;
}

void CurrentSense::calibrate(int samples) {
  if (samples <= 0) samples = 1;
  double s1 = 0.0, s2 = 0.0;
  for (int i = 0; i < samples; i++) {
    s1 += rawToVolt((uint16_t)analogRead(pin1_));
    s2 += rawToVolt((uint16_t)analogRead(pin2_));
    delayMicroseconds(50);
  }
  offset1_ = (float)(s1 / samples);
  offset2_ = (float)(s2 / samples);
}

void CurrentSense::readRaw(uint16_t& r1, uint16_t& r2) const {
  r1 = (uint16_t)analogRead(pin1_);
  r2 = (uint16_t)analogRead(pin2_);
}

void CurrentSense::readVolts(float& v1, float& v2) const {
  v1 = rawToVolt((uint16_t)analogRead(pin1_));
  v2 = rawToVolt((uint16_t)analogRead(pin2_));
}

void CurrentSense::readTwo(float& i1, float& i2) {
  const float v1 = rawToVolt((uint16_t)analogRead(pin1_));
  const float v2 = rawToVolt((uint16_t)analogRead(pin2_));
  // 注意：零偏已在 calibrate() 里测过，这里直接减
  i1 = sign1 * (v1 - offset1_) / v_per_a_;
  i2 = sign2 * (v2 - offset2_) / v_per_a_;
}

void CurrentSense::read(float& ia, float& ib, float& ic) {
  readTwo(ia, ib);
  ic = -(ia + ib);   // KCL：三相无中线电流之和为零
}

}  // namespace v3p
