/**
 * @file v3p_foc.h
 * @brief 电流环核心（阶段 3 的基础）：采样 → Clarke → Park → PI → 反Park → SVPWM
 *
 * 设计要点
 *  1. 编码器用【缓存式读取】：I2C 在 1 kHz 低频任务里读，控制环只读缓存
 *     （AS5600 一次 I2C ≈75~150 µs，比 20 kHz 周期还长 —— 见附录 G.3）
 *  2. 电流环以固定 5 kHz 运行（update() 内部按 micros() 限速）
 *  3. 可选功能全部做成开关，方便逐个实验打开/关闭：
 *       - 解耦前馈（E3.2）
 *       - 反 EMF 前馈（E3.2）
 *       - 死区电压补偿（E1.6）
 *       - 电压限幅（E3.5 要复现饱和，所以限幅值可设）
 *  4. 上电默认失能，必须显式 enable(true)
 */
#pragma once

#include "v3plab.h"

namespace v3p {

struct FocConfig {
  int   pwmA = M0_IN1, pwmB = M0_IN2, pwmC = M0_IN3;
  int   enPin = MOTOR_EN;
  int   csA = M0_OUT1_CS, csB = M0_OUT2_CS;
  int   sda = SDA_0, scl = SCL_0;
  int   chBase = 0;                 ///< LEDC 通道基址：M0 用 0，M1 用 3
  int   polePairs = POLE_PAIRS_DEFAULT;
  float vdc = VDC_DEFAULT;
  float fsw = 20000.0f;
  float rs  = RS_DEFAULT;           ///< 相电阻（E5.1 实测替换）
  float ls  = LS_DEFAULT;           ///< 相电感（E5.2 实测替换）
  float lambda = LAMBDA_DEFAULT;    ///< 磁链（E4.1 实测替换）
  float currentLimit = 1.0f;        ///< 电流限幅（A）
};

class FocMotor {
 public:
  bool begin(const FocConfig& cfg);

  /// 对齐：把转子拉到 θe=0，然后把编码器清零（必须在电机可自由转动时做）
  void alignSensor(float volts = 1.0f, uint32_t ms = 1500);

  /// 电流环：按固定周期调用（内部限速到 loopHz）
  void update();

  void setCurrentRef(float id_ref, float iq_ref) { idRef_ = id_ref; iqRef_ = iq_ref; }
  float idReference() const { return idRef_; }
  float iqReference() const { return iqRef_; }
  void setLoopHz(float hz) { loopHz_ = hz; }
  void setVoltageLimit(float v) { vLimit_ = v; }
  void enable(bool on);
  bool enabled() const { return enabled_; }

  /// 用 (ωc, R, L) 设计电流环增益：Kp = ωc·L，Ki = ωc·R
  void designCurrentLoop(float wc_rad_s);

  /// 死区电压补偿（E1.6）：本板死区由 FD6287T 内部固定，只能补偿不能设置
  void setDeadTimeComp(bool on, float td_ns = 200.0f, float eps_a = 0.05f) {
    deadTimeCompOn_ = on;
    deadTimeNs_ = td_ns;
    dtEps_ = eps_a;
  }
  bool deadTimeCompEnabled() const { return deadTimeCompOn_; }

  /// 反 EMF / 解耦前馈开关（E3.2）
  bool decouple = false;
  bool backEmfFeedforward = false;

  // ---- 无感 / I/F 强拖需要的能力（E4.5~E4.7）----
  /** true 时用 externalAngle 代替编码器角度（I/F 强拖、无感运行） */
  bool  useExternalAngle = false;
  float externalAngle = 0.0f;      ///< 外部给定电角度（rad）
  /** true 时跳过电流环，直接施加 vdRef/vqRef（电压模式，用于辨识实验） */
  bool  voltageMode = false;
  float vdRef = 0.0f, vqRef = 0.0f;

  // ---- 状态（遥测用）----
  float ia = 0, ib = 0, ic = 0;
  float id = 0, iq = 0;
  float thetaE = 0, omegaE = 0;
  float vd = 0, vq = 0;
  /** 实际施加到电机的 αβ 电压（含死区补偿后）—— 观测器的输入 */
  AlphaBeta vAlphaBeta{0.0f, 0.0f};
  /** 实测 αβ 电流（Clarke 输出）—— 观测器的输入 */
  AlphaBeta iAlphaBeta{0.0f, 0.0f};
  float vLimitScale = 1.0f;         ///< 实际施加电压 / 需求电压（<1 说明进入了限幅）
  float vBus = VDC_DEFAULT;

  FocConfig    cfg;
  PwmDriver    pwm;
  CurrentSense cs;
  As5600       enc;
  PiController pidD, pidQ;

 private:
  float  loopHz_ = 5000.0f;
  float  vLimit_ = 0.0f;
  bool   enabled_ = false;
  float  thetaOffset_ = 0.0f;       ///< 电角度偏置（align 后为 0）
  uint32_t tLoop_ = 0;
  uint32_t tEnc_  = 0;

  float  idRef_ = 0.0f, iqRef_ = 0.0f;
  bool   deadTimeCompOn_ = false;
  float  deadTimeNs_ = 200.0f;      ///< FD6287T 内部死区典型值
  float  dtEps_ = 0.05f;            ///< tanh 平滑尺度（A）

  float mechToElec(float thm) const {
    return wrap2Pi((float)cfg.polePairs * thm + thetaOffset_);
  }
};

}  // namespace v3p
