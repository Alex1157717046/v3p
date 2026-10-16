/**
 * E2.3 / E2.5 Clarke-Park 变换验证（阶段 2）
 *
 * 目标：用【已知输入】验证变换实现正确。
 *
 * 方法（实验方案 12.6 E2.3）：直流注入法
 *   ① 把转子锁定（手按住 / 用联轴器固定 / 施加固定电压矢量后断电夹紧）
 *   ② 给定固定的 id_ref、iq_ref
 *   ③ 读回 ia、ib、ic，与"由 (id,iq,θe) 反算出的期望值"对比
 *
 * 验收：
 *   给定 (id,iq) = (1,0) → ia ≈ 1 A
 *   给定 (0,1)         → 三相相差 120°
 *   全角度范围误差 < 5%
 *
 * ⚠ 幅值不变 vs 功率不变约定的差异会让结果差 √1.5 倍 —— 本工程统一用幅值不变。
 *
 * 串口命令：
 *   src enc|manual     角度来源（编码器 / 手动）
 *   theta <deg>        手动电角度
 *   idq <id> <iq>      电流给定
 *   start / stop
 *   check              打印一次"期望 vs 实测"对比表
 *   sweep              自动把 θe 扫一圈并打印误差
 */

#include <Arduino.h>
#include <Wire.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;
static const float    VDC  = VDC_DEFAULT;

static PwmDriver    pwm;
static CurrentSense cs;
static As5600       enc;
static Console      con;
static PiController pid_d, pid_q;

static bool  running = false;
static bool  useEnc  = true;
static float thetaManual = 0.0f;     // rad（电角度）
static float idRef = 0.0f, iqRef = 0.0f;

static float thE = 0, idFb = 0, iqFb = 0;
static float ia = 0, ib = 0, ic = 0;
static float iaExp = 0, ibExp = 0, icExp = 0;

static void cmd_src(int argc, char** argv) {
  if (argc < 2) return;
  useEnc = (strcmp(argv[1], "enc") == 0);
  Console::printfln("angle source = %s", useEnc ? "encoder" : "manual");
}

static void cmd_theta(int argc, char** argv) {
  const float deg = Console::argf(argv, argc, 1, 0.0f);
  thetaManual = deg * PI_F / 180.0f;
  Console::printfln("theta_manual = %.2f deg", deg);
}

static void cmd_idq(int argc, char** argv) {
  idRef = Console::argf(argv, argc, 1, idRef);
  iqRef = Console::argf(argv, argc, 2, iqRef);
  Console::printfln("idq_ref = (%.3f, %.3f) A", idRef, iqRef);
}

static void cmd_start(int, char**) {
  pid_d.reset();
  pid_q.reset();
  pwm.enable(true);
  running = true;
}
static void cmd_stop(int, char**) { running = false; pwm.coast(); }

static void cmd_check(int, char**) {
  Console::print("---- Clarke/Park 验证 ----\r\n");
  Console::printfln("theta_e = %.2f deg (src=%s)", thE * 180.0f / PI_F, useEnc ? "enc" : "manual");
  Console::printfln("ref  : id=%+.4f  iq=%+.4f A", idRef, iqRef);
  Console::printfln("fb   : id=%+.4f  iq=%+.4f A", idFb, iqFb);
  Console::printfln("ia/ib/ic meas = %+.4f %+.4f %+.4f A", ia, ib, ic);
  Console::printfln("ia/ib/ic exp  = %+.4f %+.4f %+.4f A", iaExp, ibExp, icExp);
  const float e = fmaxf(fabsf(ia - iaExp), fmaxf(fabsf(ib - ibExp), fabsf(ic - icExp)));
  const float denom = fmaxf(0.05f, fmaxf(fabsf(iaExp), fmaxf(fabsf(ibExp), fabsf(icExp))));
  Console::printfln("max abs err = %.4f A  (%.2f%%)  <- 验收 < 5%%", e, 100.0f * e / denom);
}

static void cmd_sweep(int, char**) {
  Console::print("theta(deg)  id_fb   iq_fb   ia_meas  ia_exp   err%\r\n");
  const float saved = thetaManual;
  const bool  savedSrc = useEnc;
  useEnc = false;
  for (int deg = 0; deg < 360; deg += 30) {
    thetaManual = (float)deg * PI_F / 180.0f;
    delay(120);
    const float err = fabsf(ia - iaExp);
    Console::printfln("%6d      %+.3f  %+.3f   %+.3f   %+.3f   %.1f", deg, idFb, iqFb, ia, iaExp,
                      100.0f * err / fmaxf(0.05f, fabsf(iaExp)));
  }
  thetaManual = saved;
  useEnc = savedSrc;
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E2.3/E2.5 Clarke-Park 变换验证 ===\r\n");
  Console::print("★ 做之前请把转子锁住（否则电机会转起来，电流环也稳不住）\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(20000.0f);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(1024);
  enc.begin(&Wire, SDA_0, SCL_0, 400000, 1);

  const float wc = TWO_PI_F * 300.0f;
  pid_d.kp = wc * 0.010f;  pid_d.ki = wc * 10.0f;
  pid_q.kp = wc * 0.010f;  pid_q.ki = wc * 10.0f;
  pid_d.outMin = -VDC * ONE_BY_SQRT3_F; pid_d.outMax = VDC * ONE_BY_SQRT3_F;
  pid_q.outMin = -VDC * ONE_BY_SQRT3_F; pid_q.outMax = VDC * ONE_BY_SQRT3_F;
  pid_d.intMin = pid_d.outMin; pid_d.intMax = pid_d.outMax;
  pid_q.intMin = pid_q.outMin; pid_q.intMax = pid_q.outMax;

  con.add("src",   cmd_src,   "src enc|manual");
  con.add("theta", cmd_theta, "theta <deg>");
  con.add("idq",   cmd_idq,   "idq <id> <iq>");
  con.add("start", cmd_start, "开始");
  con.add("stop",  cmd_stop,  "停止");
  con.add("check", cmd_check, "打印对比表");
  con.add("sweep", cmd_sweep, "扫一圈 θe 打印误差");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "id_ref");
  telemetry::setChannelName(1, "id_fb");
  telemetry::setChannelName(2, "iq_ref");
  telemetry::setChannelName(3, "iq_fb");
  telemetry::setChannelName(4, "ia_meas");
  telemetry::setChannelName(5, "ia_exp");

  Console::print("命令：idq 1 0 / src manual / theta 0 / start / check\r\n");
}

void loop() {
  con.poll();

  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  const float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;

  static uint32_t lastEnc = 0;
  if (now - lastEnc > 1000) { lastEnc = now; enc.update(); }

  cs.read(ia, ib, ic);

  thE = useEnc ? wrap2Pi((float)POLE_PAIRS_DEFAULT * enc.angle()) : thetaManual;

  // 期望的三相电流（由 idq 参考反算）
  {
    const AlphaBeta vExp = invPark(DQ{idRef, iqRef}, thE);
    const ABC p = abToAbc(vExp);
    iaExp = p.a; ibExp = p.b; icExp = p.c;
  }

  Duty d{0.5f, 0.5f, 0.5f};
  if (running) {
    static uint32_t tLoop = 0;
    if (now - tLoop >= 200) {          // 5 kHz 电流环
      const float dtl = (float)(now - tLoop) * 1e-6f;
      tLoop = now;

      const AlphaBeta iab = clarke(ia, ib);
      const DQ idq = park(iab, thE);
      idFb = idq.d;
      iqFb = idq.q;

      const float vd = pid_d.update(idRef - idFb, dtl);
      const float vq = pid_q.update(iqRef - iqFb, dtl);
      const AlphaBeta v = invPark(DQ{vd, vq}, thE);
      d = svpwm7(v, VDC);
      clampDuty(d);
      pwm.setDuty(d);
    }
  }

  telemetry::set(0, idRef);
  telemetry::set(1, idFb);
  telemetry::set(2, iqRef);
  telemetry::set(3, iqFb);
  telemetry::set(4, ia);
  telemetry::set(5, iaExp);
  telemetry::tick();
  (void)dt;
}
