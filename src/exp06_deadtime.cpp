/**
 * E1.6 死区：测量 + 软件补偿（阶段 1）★ 已按 V3P 实际硬件修订
 *
 * ⚠⚠ 与原方案的重大差异（务必先读）：
 *   原方案写"V3P 的 HIN/LIN# 由 MCU 分别驱动，所以死区可调、可扫描"。
 *   【这是错的】。实际原理图里 FD6287T 的 HIN1~3 与 LIN1#~3# 被接成同一组网络，
 *   每相只有 1 根 MCU PWM 线 →
 *     死区 = FD6287T 内部固定值（DT = 100/200/300 ns min/typ/max），MCU 无法设置，
 *     也【无法用软件插入死区】（无法造出"上下管同时关断"的窗口）。
 *   ⇒ "死区扫描"实验不成立（除非改板：割断 LIN# 与 HIN 的连线，飞线到空闲 GPIO）。
 *   ⇒ 但以下两件事依然可以做，而且价值很高：
 *        ① 死区【测量】—— 用示波器量 GH/GL 的真实死区（E1.7②）
 *        ② 死区【电压补偿】—— 本实验的核心
 *
 * 原理：死区造成的平均电压误差
 *        ΔV = sign(i) · (Td/Ts) · Vdc
 *   在 2208（Rs=10 Ω）上 Td=200 ns、fsw=20 kHz → ΔV = 0.12 V → 畸变电流 12 mA
 *   （占比很小，这就是"云台电机死区现象不明显"的原因；换低阻航模电机才夸张）
 *
 * 本实验怎么做：
 *   mode align：先对齐（把转子拉到 θe=0，然后把编码器清零）
 *   mode id   ：d 轴电流闭环，id 由命令给定 → 记录 (id, vd) 曲线
 *               ★ 电流从 − 扫到 + 时，vd 会在零点出现"台阶"，台阶高度 ≈ 2ΔV，
 *                 这就是死区电压误差的【实测值】
 *   comp 0|1  ：开关补偿，对比台阶是否消失
 *   mode spin ：开环旋转电压，对比补偿前后的电流过零畸变
 *
 * 串口命令：align / mode align|id|spin / id <A> / sweep 0|1 / comp 0|1 / td <ns> / eps <A> / start / stop
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
static PiController pid_d;

enum class Mode { Align, IdHold, Spin };
static Mode  mode = Mode::Align;
static bool  running = false;
static bool  sweeping = false;

static float idRef   = 0.0f;
static float tdNs    = 200.0f;     // 补偿用的死区设定值（典型 200 ns）
static float epsA    = 0.05f;      // tanh 平滑尺度
static bool  compOn  = false;
static float fsw     = 20000.0f;
static float fe      = 1.0f;       // spin 模式电频率
static float thetaE  = 0.0f;
static uint32_t tPrev = 0;

static float idFb = 0, iqFb = 0, vd = 0, vq = 0, idErr = 0;

// ------------------------------------------------------------------ 工具

static void electricalAngle(float& th, float& we) {
  th = wrap2Pi((float)POLE_PAIRS_DEFAULT * enc.angle());
  we = (float)POLE_PAIRS_DEFAULT * enc.velocity();
}

/// 每相死区电压补偿
static void applyComp(AlphaBeta& v, float ia, float ib, float ic) {
  if (!compOn) return;
  const float dV = (tdNs * 1e-9f) * fsw * VDC;   // (Td/Ts)·Vdc
  const ABC p = abToAbc(v);
  const ABC c{deadTimeComp(ia, epsA, tdNs * 1e-9f, 1.0f / fsw, VDC),
              deadTimeComp(ib, epsA, tdNs * 1e-9f, 1.0f / fsw, VDC),
              deadTimeComp(ic, epsA, tdNs * 1e-9f, 1.0f / fsw, VDC)};
  // 参考电压 = 期望电压 + 补偿项（抵消死区造成的损失）
  const float va = p.a + c.a, vb = p.b + c.b, vc = p.c + c.c;
  v.alpha = va;
  v.beta  = (vb - vc) * ONE_BY_SQRT3_F;
  (void)dV;
}

// ------------------------------------------------------------------ 命令

static void cmd_align(int, char**) {
  Console::print("aligning... 施加 θe=0 电压矢量，把转子拉到 d 轴\r\n");
  pwm.enable(false);
  const AlphaBeta v{1.0f, 0.0f};        // 1 V 左右，小电流
  Duty d = svpwm7(v, VDC);
  clampDuty(d);
  pwm.setDuty(d);
  pwm.enable(true);
  delay(1500);
  pwm.enable(false);
  pwm.setDuty(0.5f, 0.5f, 0.5f);
  enc.setZero();
  Console::printfln("align done. encoder zero set. (θm=%.4f rad)", enc.angle());
  mode = Mode::Align;
  running = false;
}

static void cmd_mode(int argc, char** argv) {
  if (argc < 2) return;
  if      (!strcmp(argv[1], "align")) mode = Mode::Align;
  else if (!strcmp(argv[1], "id"))    mode = Mode::IdHold;
  else if (!strcmp(argv[1], "spin"))  mode = Mode::Spin;
  Console::printfln("mode = %s", argv[1]);
}

static void cmd_id(int argc, char** argv) {
  idRef = Console::argf(argv, argc, 1, idRef);
  idRef = fmaxf(-0.6f, fminf(0.6f, idRef));
  Console::printfln("id_ref = %.3f A", idRef);
}

static void cmd_sweep(int argc, char** argv) {
  sweeping = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("sweep %s", sweeping ? "ON (-0.6 → +0.6 A)" : "OFF");
}

static void cmd_comp(int argc, char** argv) {
  compOn = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("compensation %s (td=%.0f ns, eps=%.3f A)", compOn ? "ON" : "OFF", tdNs, epsA);
}

static void cmd_td(int argc, char** argv) {
  tdNs = Console::argf(argv, argc, 1, tdNs);
  Console::printfln("td = %.0f ns  → ΔV = %.4f V", tdNs, tdNs * 1e-9f * fsw * VDC);
}

static void cmd_eps(int argc, char** argv) { epsA = Console::argf(argv, argc, 1, epsA); }

static void cmd_fsw(int argc, char** argv) {
  fsw = Console::argf(argv, argc, 1, fsw);
  pwm.setFrequency(fsw);
  Console::printfln("fsw = %.0f Hz → ΔV = %.4f V", fsw, tdNs * 1e-9f * fsw * VDC);
}

static void cmd_start(int, char**) {
  pid_d.reset();
  tPrev = micros();
  pwm.enable(true);
  running = true;
}

static void cmd_stop(int, char**) {
  running = false;
  pwm.coast();
}

// ------------------------------------------------------------------ 主体

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);

  Console::print("\r\n=== E1.6 死区：测量 + 软件补偿（已按 V3P 实际硬件修订）===\r\n");
  Console::print("★ 本板死区由 FD6287T 内部固定（typ 200 ns），MCU 不能设置，只能补偿。\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(fsw);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(1024);

  if (!enc.begin(&Wire, SDA_0, SCL_0, 400000, 1)) {
    Console::print("AS5600 M0 FAIL —— 请检查编码器接线后再做本实验\r\n");
  }

  // d 轴电流环：2208 的 R≈10Ω、L≈10mH
  // 按 Kp = ωc·L、Ki = ωc·R 取 ωc = 2π·300 rad/s
  const float wc = TWO_PI_F * 300.0f;
  pid_d.kp = wc * 0.010f;         // ≈ 18.8 V/A
  pid_d.ki = wc * 10.0f;          // ≈ 18850 V/(A·s)
  pid_d.outMin = -VDC * ONE_BY_SQRT3_F;
  pid_d.outMax =  VDC * ONE_BY_SQRT3_F;
  pid_d.intMin = -VDC * ONE_BY_SQRT3_F;
  pid_d.intMax =  VDC * ONE_BY_SQRT3_F;
  pid_d.aw = AntiWindup::Clamp;

  con.add("align", cmd_align, "对齐并把编码器清零");
  con.add("mode",  cmd_mode,  "mode align|id|spin");
  con.add("id",    cmd_id,    "id <A> d轴电流给定");
  con.add("sweep", cmd_sweep, "sweep 0|1 自动扫 id");
  con.add("comp",  cmd_comp,  "comp 0|1 死区补偿开关");
  con.add("td",    cmd_td,    "td <ns> 补偿用死区值");
  con.add("eps",   cmd_eps,   "eps <A> tanh 平滑尺度");
  con.add("fsw",   cmd_fsw,   "fsw <Hz>");
  con.add("start", cmd_start, "开始闭环");
  con.add("stop",  cmd_stop,  "停止");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "id_ref");
  telemetry::setChannelName(1, "id_fb");
  telemetry::setChannelName(2, "vd");
  telemetry::setChannelName(3, "vq");
  telemetry::setChannelName(4, "comp");
  telemetry::setChannelName(5, "theta_e");

  Console::print("流程：align → mode id → start → sweep 1  → 看 vd–id 台阶 → comp 1 再看\r\n");
}

void loop() {
  con.poll();

  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  // 编码器 1 kHz 更新
  static uint32_t lastEnc = 0;
  if (now - lastEnc > 1000) { lastEnc = now; enc.update(); }

  float ia, ib, ic;
  cs.read(ia, ib, ic);

  float th = 0, we = 0;
  electricalAngle(th, we);

  // 自动扫描 id
  if (running && sweeping && mode == Mode::IdHold) {
    static uint32_t tLast = 0;
    if (now - tLast > 1500000u) {
      tLast = now;
      idRef += 0.15f;
      if (idRef > 0.6001f) idRef = -0.60f;
      Console::printfln("[sweep] id_ref=%+.2f A", idRef);
    }
  }

  Duty d{0.5f, 0.5f, 0.5f};
  if (running) {
    // 电流环以 5 kHz 运行
    static uint32_t tLoop = 0;
    if (now - tLoop >= 200) {
      const float dtl = (float)(now - tLoop) * 1e-6f;
      tLoop = now;

      const AlphaBeta iab = clarke(ia, ib);
      const DQ idq = park(iab, th);
      idFb = idq.d;
      iqFb = idq.q;

      if (mode == Mode::IdHold) {
        idErr = idRef - idFb;
        vd = pid_d.update(idErr, dtl);
        vq = 0.0f;
      } else if (mode == Mode::Spin) {
        // 开环旋转：固定电压幅值，看补偿对电流过零畸变的影响
        thetaE = wrap2Pi(thetaE + TWO_PI_F * fe * dtl);
        vd = 1.5f;
        vq = 0.0f;
        th = thetaE;
      } else {
        vd = 0.0f; vq = 0.0f;
      }

      AlphaBeta v = invPark(DQ{vd, vq}, th);
      applyComp(v, ia, ib, ic);
      d = svpwm7(v, VDC);
      clampDuty(d);
      pwm.setDuty(d);
    }
  }

  telemetry::set(0, idRef);
  telemetry::set(1, idFb);
  telemetry::set(2, vd);
  telemetry::set(3, vq);
  telemetry::set(4, compOn ? 1.0f : 0.0f);
  telemetry::set(5, th);
  telemetry::tick();
  (void)d;
}
