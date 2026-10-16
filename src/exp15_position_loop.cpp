/**
 * E3.4 位置环与前馈（阶段 3）
 *
 * 目标：实现高精度位置跟踪 —— 这就是【云台】的核心技术。
 * 验收：纯 P 位置环有稳态误差（斜坡跟踪时）；加【速度前馈】后斜坡跟踪误差 → 接近 0；
 *       加【加速度前馈】后正弦跟踪相位滞后明显减小。
 *
 * 结构：位置 P(+FF) → 速度 PI → 电流 PI(内环) → SVPWM
 *
 * 串口命令：
 *   align              对齐（把当前机械角设为 0）
 *   pos <deg>          位置给定
 *   kp <1/s>           位置环比例（单位 1/s）
 *   kvff <0|1>         速度前馈开关
 *   kaff <0|1>         加速度前馈开关
 *   ramp <deg/s>       斜坡跟踪测试（测"有没有前馈"的差别最明显）
 *   sine <Hz> <deg>    正弦跟踪测试
 *   start / stop / report
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static FocMotor    m;
static Console     con;
static PiController pidSpeed;

static bool  running = false;
static float kpPos = 12.0f;         // 1/s
static bool  kvff = false, kaff = false;
static float posRef = 0.0f;         // rad（机械）
static float rampRate = 0.0f;       // rad/s
static float sineHz = 0.0f, sineAmp = 0.0f;
static float errPeak = 0.0f, errSum = 0.0f, errN = 0;

static void designSpeedLoop(float hz) {
  const float wc = TWO_PI_F * hz;
  pidSpeed.kp = wc * 2.0e-6f / 0.083f;
  pidSpeed.ki = wc * pidSpeed.kp * 0.5f;
  pidSpeed.outMin = -m.cfg.currentLimit;
  pidSpeed.outMax =  m.cfg.currentLimit;
  pidSpeed.intMin = pidSpeed.outMin;
  pidSpeed.intMax = pidSpeed.outMax;
  pidSpeed.aw = AntiWindup::Clamp;
}

static void cmd_align(int, char**) {
  Console::print("aligning...\r\n");
  m.alignSensor(1.0f, 1500);
  m.enc.setZero();
  posRef = 0.0f;
  Console::print("done. 当前位置 = 0\r\n");
}

static void cmd_start(int, char**) {
  pidSpeed.reset();
  m.enable(true);
  running = true;
  errPeak = 0; errSum = 0; errN = 0;
}
static void cmd_stop(int, char**) { running = false; m.enable(false); }

static void cmd_pos(int argc, char** argv) {
  posRef = Console::argf(argv, argc, 1, 0.0f) * PI_F / 180.0f;
  rampRate = 0.0f; sineHz = 0.0f;
  errPeak = 0; errSum = 0; errN = 0;
  Console::printfln("pos_ref = %.2f deg", posRef * 180.0f / PI_F);
}

static void cmd_kp(int argc, char** argv) {
  kpPos = Console::argf(argv, argc, 1, kpPos);
  Console::printfln("Kp_pos = %.2f 1/s", kpPos);
}

static void cmd_kvff(int argc, char** argv) {
  kvff = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("速度前馈 %s", kvff ? "ON" : "OFF");
}
static void cmd_kaff(int argc, char** argv) {
  kaff = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("加速度前馈 %s", kaff ? "ON" : "OFF");
}

static void cmd_ramp(int argc, char** argv) {
  rampRate = Console::argf(argv, argc, 1, 90.0f) * PI_F / 180.0f;   // deg/s → rad/s
  sineHz = 0.0f;
  errPeak = 0; errSum = 0; errN = 0;
  Console::printfln("斜坡跟踪 %.1f deg/s（对比 kvff 0/1 的稳态误差）", rampRate * 180.0f / PI_F);
}

static void cmd_sine(int argc, char** argv) {
  sineHz  = Console::argf(argv, argc, 1, 0.5f);
  sineAmp = Console::argf(argv, argc, 2, 30.0f) * PI_F / 180.0f;
  rampRate = 0.0f;
  errPeak = 0; errSum = 0; errN = 0;
  Console::printfln("正弦跟踪 f=%.2f Hz A=%.1f deg", sineHz, sineAmp * 180.0f / PI_F);
}

static void cmd_report(int, char**) {
  const float rms = (errN > 0) ? sqrtf(errSum / (float)errN) : 0.0f;
  Console::print("---- 位置跟踪指标 ----\r\n");
  Console::printfln("kvff=%d kaff=%d  Kp_pos=%.2f", kvff ? 1 : 0, kaff ? 1 : 0, kpPos);
  Console::printfln("误差峰值 = %.3f deg   误差 RMS = %.3f deg", errPeak * 180.0f / PI_F,
                    rms * 180.0f / PI_F);
  Console::printfln("当前位置 = %.2f deg", m.enc.angle() * 180.0f / PI_F);
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E3.4 位置环与前馈 ===\r\n");

  FocConfig c;
  c.fsw = 20000.0f;
  c.currentLimit = 1.0f;
  m.begin(c);
  m.setLoopHz(5000.0f);
  m.designCurrentLoop(TWO_PI_F * 300.0f);
  designSpeedLoop(8.0f);

  con.add("align",  cmd_align,  "对齐并清零");
  con.add("start",  cmd_start,  "启动");
  con.add("stop",   cmd_stop,   "停止");
  con.add("pos",    cmd_pos,    "pos <deg>");
  con.add("kp",     cmd_kp,     "kp <1/s>");
  con.add("kvff",   cmd_kvff,   "kvff 0|1 速度前馈");
  con.add("kaff",   cmd_kaff,   "kaff 0|1 加速度前馈");
  con.add("ramp",   cmd_ramp,   "ramp <deg/s> 斜坡跟踪");
  con.add("sine",   cmd_sine,   "sine <Hz> <deg> 正弦跟踪");
  con.add("report", cmd_report, "打印误差指标");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "pos_ref");
  telemetry::setChannelName(1, "pos");
  telemetry::setChannelName(2, "err_deg");
  telemetry::setChannelName(3, "omega_ref");
  telemetry::setChannelName(4, "omega");
  telemetry::setChannelName(5, "iq");

  Console::print("建议对比：ramp 90 + kvff 0 → report ；再 kvff 1 → report\r\n");
}

void loop() {
  con.poll();

  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  // ---- 轨迹生成 ----
  static float tTraj = 0.0f;
  static float prevRef = 0.0f, prevVel = 0.0f;
  tTraj += dt;

  float spdFF = 0.0f, accFF = 0.0f;
  if (sineHz > 0.0f) {
    const float w = TWO_PI_F * sineHz;
    posRef = sineAmp * sinf(w * tTraj);
    spdFF  = sineAmp * w * cosf(w * tTraj);
    accFF  = -sineAmp * w * w * sinf(w * tTraj);
  } else if (rampRate != 0.0f) {
    posRef += rampRate * dt;
    spdFF = rampRate;
    accFF = 0.0f;
  }
  prevRef = posRef; prevVel = spdFF;

  float iqRef = 0.0f;
  const float posM = m.enc.angle();
  if (running) {
    const float e = posRef - posM;
    float spdRefCmd = kpPos * e;
    if (kvff) spdRefCmd += spdFF;
    if (kaff) spdRefCmd += accFF * 0.02f;      // 加速度前馈增益（需整定）
    iqRef = pidSpeed.update(spdRefCmd - m.enc.velocity(), dt);
    m.setCurrentRef(0.0f, iqRef);
  } else {
    m.setCurrentRef(0.0f, 0.0f);
  }

  m.update();

  if (running) {
    const float e = fabsf(posRef - m.enc.angle());
    if (e > errPeak) errPeak = e;
    errSum += e * e;
    errN++;
  }

  telemetry::set(0, posRef);
  telemetry::set(1, posM);
  telemetry::set(2, (posRef - posM) * 180.0f / PI_F);
  telemetry::set(3, spdFF);
  telemetry::set(4, m.enc.velocity());
  telemetry::set(5, m.iq);
  telemetry::tick();
  (void)prevRef; (void)prevVel;
}
