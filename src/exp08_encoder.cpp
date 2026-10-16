/**
 * E2.1 AS5600 标定与评估（阶段 2）
 *
 * 目标：得到干净、准确的电角度。
 * 验收：静止角度噪声 < 0.1°（0.0017 rad）；给出非线性/AGC 曲线；低速速度噪声量化。
 *
 * 步骤（对应实验方案）：
 *   ① 对齐零位（用 E1.6 的 align，或直接 zero 命令）
 *   ② 定方向（dir 1|-1）
 *   ③ 测静态噪声（标准差，单位度）→ stat 5
 *   ④ 用手／另一台电机匀速拖动，看非线性（遥测里看 angle 与 raw 的关系）
 *   ⑤ 用编码器差分测速度噪声
 *
 * 串口命令：
 *   zero            当前角度清零
 *   dir 1|-1        设方向
 *   stat <sec>      统计 N 秒：raw 噪声、角度噪声(度)、速度噪声
 *   rate <Hz>       I2C 读取率（默认 1000）
 *   lpf <Hz>        速度低通截止频率
 *   mag             读磁铁状态（MD/ML/MH + AGC）
 */

#include <Arduino.h>
#include <Wire.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static As5600  enc0, enc1;
static Console con;

static uint32_t readRateHz = 1000;
static float    lastStatStdRaw = 0, lastStatStdDeg = 0, lastStatStdVel = 0;

static void cmd_zero(int, char**) {
  enc0.setZero();
  Console::printfln("zero set. angle=%.4f rad", enc0.angle());
}

static void cmd_dir(int argc, char** argv) {
  const int d = (int)Console::argi(argv, argc, 1, 1);
  enc0.setDirection(d);
  Console::printfln("dir = %d", enc0.direction());
}

static void cmd_rate(int argc, char** argv) {
  readRateHz = (uint32_t)Console::argi(argv, argc, 1, (int32_t)readRateHz);
  if (readRateHz < 1) readRateHz = 1;
  Console::printfln("read rate = %u Hz", (unsigned)readRateHz);
}

static void cmd_lpf(int argc, char** argv) {
  const float fc = Console::argf(argv, argc, 1, 200.0f);
  enc0.setVelocityLpf(fc, (float)readRateHz);
  Console::printfln("velocity LPF fc = %.1f Hz", fc);
}

static void cmd_mag(int, char**) {
  Console::printfln("M0: MD=%d ML=%d MH=%d AGC=%u", enc0.magnetDetected() ? 1 : 0,
                    enc0.magnetWeak() ? 1 : 0, enc0.magnetStrong() ? 1 : 0, enc0.agc());
  Console::printfln("M1: MD=%d ML=%d MH=%d AGC=%u", enc1.magnetDetected() ? 1 : 0,
                    enc1.magnetWeak() ? 1 : 0, enc1.magnetStrong() ? 1 : 0, enc1.agc());
}

static void cmd_stat(int argc, char** argv) {
  const float sec = Console::argf(argv, argc, 1, 5.0f);
  const uint32_t t0 = millis();
  const uint32_t nTarget = (uint32_t)(sec * (float)readRateHz);

  double sRaw = 0, s2Raw = 0, sAng = 0, s2Ang = 0, sVel = 0, s2Vel = 0;
  uint32_t n = 0;
  while (millis() - t0 < (uint32_t)(sec * 1000.0f) && n < nTarget) {
    enc0.update();
    const double r = (double)enc0.raw();
    const double a = (double)enc0.angle();
    const double v = (double)enc0.velocity();
    sRaw += r; s2Raw += r * r;
    sAng += a; s2Ang += a * a;
    sVel += v; s2Vel += v * v;
    n++;
    delayMicroseconds(1000000u / readRateHz);
  }
  if (n < 2) { Console::print("too few samples\r\n"); return; }

  const double mRaw = sRaw / n, mAng = sAng / n, mVel = sVel / n;
  const double vRaw = s2Raw / n - mRaw * mRaw;
  const double vAng = s2Ang / n - mAng * mAng;
  const double vVel = s2Vel / n - mVel * mVel;

  lastStatStdRaw = (float)sqrt(vRaw > 0 ? vRaw : 0);
  lastStatStdDeg = (float)(sqrt(vAng > 0 ? vAng : 0) * 180.0 / PI);
  lastStatStdVel = (float)sqrt(vVel > 0 ? vVel : 0);

  Console::printfln("samples = %u", (unsigned)n);
  Console::printfln("raw    : std = %.4f LSB  (1 LSB = %.4f deg)", lastStatStdRaw, 360.0f / 4096.0f);
  Console::printfln("angle  : std = %.5f deg   <- 验收 < 0.1 deg", lastStatStdDeg);
  Console::printfln("vel    : std = %.4f rad/s (%.3f rpm)", lastStatStdVel,
                    lastStatStdVel * 60.0f / TWO_PI_F);
  Console::printfln("AGC = %u  (越小说明磁场越强)", enc0.agc());
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E2.1 AS5600 标定与评估 ===\r\n");

  const bool ok0 = enc0.begin(&Wire, SDA_0, SCL_0, 400000, 1);
  const bool ok1 = enc1.begin(&Wire1, SDA_1, SCL_1, 400000, 1);
  Console::printfln("M0: %s   M1: %s", ok0 ? "OK" : "FAIL", ok1 ? "OK" : "FAIL");
  enc0.setVelocityLpf(200.0f, (float)readRateHz);

  con.add("zero", cmd_zero, "当前角度清零");
  con.add("dir",  cmd_dir,  "dir 1|-1");
  con.add("stat", cmd_stat, "stat <sec> 统计噪声");
  con.add("rate", cmd_rate, "rate <Hz> I2C 读取率");
  con.add("lpf",  cmd_lpf,  "lpf <Hz> 速度低通");
  con.add("mag",  cmd_mag,  "读磁铁状态与 AGC");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "raw");
  telemetry::setChannelName(1, "angle_rad");
  telemetry::setChannelName(2, "angle_deg");
  telemetry::setChannelName(3, "vel_rad_s");
  telemetry::setChannelName(4, "agc");
  telemetry::setChannelName(5, "std_deg");

  Console::print("命令：stat 5 / mag / zero / dir 1 / rate 1000\r\n");
}

void loop() {
  con.poll();

  static uint32_t last = 0;
  const uint32_t now = micros();
  if (now - last >= 1000000u / readRateHz) {
    last = now;
    enc0.update();
  }

  telemetry::set(0, (float)enc0.raw());
  telemetry::set(1, enc0.angle());
  telemetry::set(2, enc0.angle() * 180.0f / PI_F);
  telemetry::set(3, enc0.velocity());
  telemetry::set(4, (float)enc0.agc());
  telemetry::set(5, lastStatStdDeg);
  telemetry::tick();
}
