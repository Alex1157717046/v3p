/**
 * E0.1 硬件清点与自检（阶段 0）
 *
 * 目标：确认手上这块 V3P 的真实配置，建立"硬件档案"。
 * 本实验【不会让电机转】——驱动板默认失能，只做读取与自检。
 *
 * 输出：
 *   1. 板级档案（引脚/分流/增益/量程/极对数）
 *   2. 两路 AS5600 连通性 + 磁铁诊断
 *   3. 电流采样零偏与噪声（std）
 *   4. PWM 占空比输出（可用示波器在 MOS 栅极/相线看波形）
 *   5. 二进制遥测（可用 tools/vofa_view.py 看波形）
 *
 * 串口命令（波特率 921600）：
 *   report            重新打印硬件档案
 *   cal               重新标定电流零偏（必须无电流！）
 *   en 0|1            使能/失能驱动板（使能前请确认三相占空比=0.5,0.5,0.5）
 *   duty a b c        设三相占空比 0~1
 *   fsw 20000         改 PWM 频率
 *   scan              扫描 I2C 总线
 *   stream 0|1        开/关二进制遥测
 */

#include <Arduino.h>
#include <Wire.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;
static const float    FSW_DEFAULT = 20000.0f;

static PwmDriver    pwm0;
static CurrentSense cs0;
static As5600       enc0;
static As5600       enc1;
static Console      con;

static bool streaming = true;
static float dutyA = 0.5f, dutyB = 0.5f, dutyC = 0.5f;

// ---------------------------------------------------------------- 命令实现

static void cmd_report(int, char**) {
  Console::print("==== DengFOC V3P 硬件档案 ====\r\n");
  Console::printfln("board          : ESP32 (LOLIN32 Lite) 背插 V3P");
  Console::printfln("功率级          : 6x WSD4070DN + FD6287T / 路（HIN 与 LIN# 板上并联）");
  Console::printfln("M0 PWM pins    : %d,%d,%d   M1: %d,%d,%d  EN=%d",
                    M0_IN1, M0_IN2, M0_IN3, M1_IN1, M1_IN2, M1_IN3, MOTOR_EN);
  Console::printfln("电流采样        : M0=%d,%d  M1=%d,%d", M0_OUT1_CS, M0_OUT2_CS,
                    M1_OUT1_CS, M1_OUT2_CS);
  Console::printfln("R_shunt        : %.4f ohm   INA gain: %.0f V/V", R_SHUNT, INA_GAIN);
  Console::printfln("灵敏度          : %.3f V/A   量程: +-%.2f A", CSA_V_PER_A, I_FULL_SCALE);
  Console::printfln("ADC LSB        : %.2f mA", 1000.0f * (ADC_VREF / 4095.0f) / CSA_V_PER_A);
  Console::printfln("编码器 I2C      : M0 SDA=%d SCL=%d ; M1 SDA=%d SCL=%d", SDA_0, SCL_0, SDA_1, SCL_1);
  Console::printfln("极对数 p        : %d   默认母线: %.1f V", POLE_PAIRS_DEFAULT, VDC_DEFAULT);
  Console::printfln("PWM 频率        : %.0f Hz (%u bit)", pwm0.frequency(), (unsigned)pwm0.maxDuty());
}

static void cmd_cal(int, char**) {
  Console::print("calibrating offsets (must be ZERO current)...\r\n");
  cs0.calibrate(1024);
  Console::printfln("offset1 = %.4f V  offset2 = %.4f V", cs0.offset1(), cs0.offset2());
}

static void cmd_en(int argc, char** argv) {
  const int on = (int)Console::argi(argv, argc, 1, 0);
  pwm0.enable(on != 0);
  Console::printfln("driver %s", on ? "ENABLED" : "disabled");
}

static void cmd_duty(int argc, char** argv) {
  dutyA = Console::argf(argv, argc, 1, dutyA);
  dutyB = Console::argf(argv, argc, 2, dutyB);
  dutyC = Console::argf(argv, argc, 3, dutyC);
  pwm0.setDuty(dutyA, dutyB, dutyC);
  Console::printfln("duty = %.3f %.3f %.3f", dutyA, dutyB, dutyC);
}

static void cmd_fsw(int argc, char** argv) {
  const float f = Console::argf(argv, argc, 1, 20000.0f);
  pwm0.setFrequency(f);
  Console::printfln("fsw = %.0f Hz  (%u bit, max duty %u)",
                    pwm0.frequency(), (unsigned)pwm0.maxDuty() - 0u, (unsigned)pwm0.maxDuty());
}

static void cmd_scan(int, char**) {
  Console::print("I2C scan:\r\n");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Console::printfln("  found 0x%02X", a);
  }
}

static void cmd_stream(int argc, char** argv) {
  streaming = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("stream %s", streaming ? "ON" : "OFF");
}

// ---------------------------------------------------------------- setup/loop

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(200);

  Console::print("\r\n\r\n=== E0.1 DengFOC V3P 硬件清点与自检 ===\r\n");

  // --- PWM：先建立 50/50/50（三相电压为零，最安全），但【不使能】驱动板 ---
  pwm0.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm0.setFrequency(FSW_DEFAULT);
  pwm0.setDuty(0.5f, 0.5f, 0.5f);

  // --- 电流采样：无电流时标定零偏 ---
  cs0.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs0.calibrate(1024);

  // --- 编码器 ---
  const bool ok0 = enc0.begin(&Wire, SDA_0, SCL_0, 400000, 1);
  const bool ok1 = enc1.begin(&Wire1, SDA_1, SCL_1, 400000, 1);
  Console::printfln("AS5600 M0: %s   M1: %s", ok0 ? "OK" : "FAIL", ok1 ? "OK" : "FAIL");

  cmd_report(0, nullptr);

  // --- 电流采样噪声（静止，无电流）---
  {
    double s = 0, s2 = 0;
    const int N = 2000;
    for (int i = 0; i < N; i++) {
      float i1, i2;
      cs0.readTwo(i1, i2);
      s += i1;
      s2 += (double)i1 * i1;
      delayMicroseconds(200);
    }
    const double mean = s / N;
    const double var = s2 / N - mean * mean;
    Console::printfln("noise(ch1): mean=%.4f A  std=%.4f A (%.1f mA)",
                      (float)mean, (float)sqrt(var > 0 ? var : 0), 1000.0f * (float)sqrt(var > 0 ? var : 0));
  }

  con.add("report", cmd_report, "打印硬件档案");
  con.add("cal",    cmd_cal,    "重新标定电流零偏");
  con.add("en",     cmd_en,     "en 0|1 使能驱动板");
  con.add("duty",   cmd_duty,   "duty a b c");
  con.add("fsw",    cmd_fsw,    "fsw <Hz>");
  con.add("scan",   cmd_scan,   "扫描 I2C");
  con.add("stream", cmd_stream, "stream 0|1");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "i_b");
  telemetry::setChannelName(2, "i_c");
  telemetry::setChannelName(3, "theta");
  telemetry::setChannelName(4, "vbus_raw");
  telemetry::setChannelName(5, "agc");

  Console::print("ready. 输入 help 看命令。\r\n");
}

void loop() {
  con.poll();

  static uint32_t t0 = 0;
  static float    tSec = 0.0f;
  const uint32_t now = micros();
  if (t0 == 0) t0 = now;
  tSec = (float)(now - t0) * 1e-6f;

  // --- 读编码器（低频即可；ISR 化的做法见 E2.4）---
  static uint32_t lastEnc = 0;
  if (now - lastEnc > 1000) {          // 1 kHz
    lastEnc = now;
    enc0.update();
    enc1.update();
  }

  // --- 电流 + 遥测变量 ---
  float ia, ib, ic;
  cs0.read(ia, ib, ic);
  telemetry::set(0, ia);
  telemetry::set(1, ib);
  telemetry::set(2, ic);
  telemetry::set(3, enc0.angle());
  telemetry::set(4, (float)analogRead(POWER_ADC));
  telemetry::set(5, (float)enc0.agc());

  if (streaming) telemetry::tick();
  (void)tSec;
}
