#include "v3p_foc.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

namespace v3p {

bool FocMotor::begin(const FocConfig& c) {
  cfg = c;

  pwm.begin(cfg.pwmA, cfg.pwmB, cfg.pwmC, cfg.enPin, cfg.chBase);
  pwm.setFrequency(cfg.fsw);
  // 上电安全状态：三相 50% = 零电压
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(cfg.csA, cfg.csB);
  cs.calibrate(1024);

  const bool okEnc = enc.begin(&Wire, cfg.sda, cfg.scl, 400000, 1);
  enc.setVelocityLpf(300.0f, 1000.0f);

  vBus  = cfg.vdc;
  vLimit_ = cfg.vdc * ONE_BY_SQRT3_F * 0.95f;
  designCurrentLoop(TWO_PI_F * 300.0f);
  return okEnc;
}

void FocMotor::designCurrentLoop(float wc_rad_s) {
  // 一阶对象 i(s)/v(s) = 1/(L s + R) → Kp = ωc·L，Ki = ωc·R
  pidD.kp = wc_rad_s * cfg.ls;
  pidD.ki = wc_rad_s * cfg.rs;
  pidQ.kp = pidD.kp;
  pidQ.ki = pidD.ki;

  const float vlim = vLimit_;
  pidD.outMin = -vlim; pidD.outMax = vlim;
  pidQ.outMin = -vlim; pidQ.outMax = vlim;
  pidD.intMin = -vlim; pidD.intMax = vlim;
  pidQ.intMin = -vlim; pidQ.intMax = vlim;
}

void FocMotor::enable(bool on) {
  if (on) {
    pidD.reset();
    pidQ.reset();
    pwm.setDuty(0.5f, 0.5f, 0.5f);
    pwm.enable(true);
  } else {
    pwm.coast();
  }
  enabled_ = on;
}

void FocMotor::alignSensor(float volts, uint32_t ms) {
  // 施加 θe = 0 的电压矢量，把转子拉到 d 轴
  const AlphaBeta v{volts, 0.0f};
  Duty d = svpwm7(v, cfg.vdc);
  clampDuty(d);
  pwm.setDuty(d);
  pwm.enable(true);
  delay(ms);
  pwm.enable(false);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  enc.update();
  enc.setZero();
  thetaOffset_ = 0.0f;
  enabled_ = false;
}

void FocMotor::update() {
  const uint32_t now = micros();

  // ---- 编码器：1 kHz 低频读取（I2C 不进控制环）----
  if (now - tEnc_ >= 1000) {
    tEnc_ = now;
    enc.update();
  }

  // ---- 电流采样 ----
  cs.read(ia, ib, ic);

  // ---- 控制环（默认 5 kHz）----
  const uint32_t periodUs = (uint32_t)(1e6f / loopHz_);
  if (now - tLoop_ < periodUs) return;
  const float dt = (float)(now - tLoop_) * 1e-6f;
  tLoop_ = now;
  if (dt <= 0.0f || dt > 0.05f) return;

  const float thm = enc.angle();
  thetaE = useExternalAngle ? wrapPi(externalAngle) : mechToElec(thm);
  // 无感运行时 omegaE 只是"参考"；外部角度模式下用编码器差分仍可当真值用
  omegaE = (float)cfg.polePairs * enc.velocity();

  const AlphaBeta iab = clarke(ia, ib);
  iAlphaBeta = iab;
  const DQ idq = park(iab, thetaE);
  id = idq.d;
  iq = idq.q;

  // ---- 电流限幅（作用在参考上，避免 PI 饱和）----
  float idr = idRef_;
  float iqr = iqRef_;
  const float imag = sqrtf(idr * idr + iqr * iqr);
  if (imag > cfg.currentLimit && imag > 1e-6f) {
    const float k = cfg.currentLimit / imag;
    idr *= k;
    iqr *= k;
  }

  float vdCmd, vqCmd;
  if (voltageMode) {
    // 电压模式：直接施加给定电压（E4.1/E5.x 辨识用）
    vdCmd = vdRef;
    vqCmd = vqRef;
  } else {
    vdCmd = pidD.update(idr - id, dt);
    vqCmd = pidQ.update(iqr - iq, dt);
  }

  // ---- 解耦 / 反 EMF 前馈（E3.2）----
  if (decouple) {
    vdCmd -= omegaE * cfg.ls * iq;
    vqCmd += omegaE * cfg.ls * id;
  }
  if (backEmfFeedforward) {
    vqCmd += omegaE * cfg.lambda;
  }

  // ---- 电压限幅（保留比例，便于观察"进入饱和"）----
  const float vmag = sqrtf(vdCmd * vdCmd + vqCmd * vqCmd);
  vLimitScale = 1.0f;
  if (vmag > vLimit_ && vmag > 1e-6f) {
    vLimitScale = vLimit_ / vmag;
    vdCmd *= vLimitScale;
    vqCmd *= vLimitScale;
  }

  vd = vdCmd;
  vq = vqCmd;

  AlphaBeta v = invPark(DQ{vdCmd, vqCmd}, thetaE);
  // ---- 死区电压补偿（E1.6）----
  if (deadTimeCompOn_) {
    const ABC p = abToAbc(v);
    const float ts = 1.0f / cfg.fsw;
    const float td = deadTimeNs_ * 1e-9f;
    const ABC comp{deadTimeComp(ia, dtEps_, td, ts, cfg.vdc),
                   deadTimeComp(ib, dtEps_, td, ts, cfg.vdc),
                   deadTimeComp(ic, dtEps_, td, ts, cfg.vdc)};
    const float va = p.a + comp.a, vb = p.b + comp.b, vc = p.c + comp.c;
    v.alpha = va;
    v.beta  = (vb - vc) * ONE_BY_SQRT3_F;
  }

  vAlphaBeta = v;   // 给观测器用（观测器必须知道"实际施加了什么电压"）

  Duty d = svpwm7(v, cfg.vdc);
  clampDuty(d);
  if (enabled_) pwm.setDuty(d);
}

}  // namespace v3p
