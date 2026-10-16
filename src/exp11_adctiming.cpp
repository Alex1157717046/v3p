/**
 * E2.4 ADC 采样时序与算法耗时（阶段 2）
 *
 * 目标：
 *   ① 搞清"采样发生在 PWM 周期的哪个时刻"（用 GPIO 翻转标记，与 PWM 同屏看）
 *   ② 测 ADC 转换 + 算法耗时（WCET），算出占周期比例
 *   ③ 对比 analogRead() 与 ESP-IDF adc1_get_raw() 的速度差
 *
 * 验收：采样点落在预期位置；给出算法耗时（µs）与占周期比例。
 *
 * ⚠ 陷阱：ESP32 的 Arduino analogRead() 慢且与非 PWM 外设争用；
 *         官方例程用 analogRead，但高性能场合应换 ADC DMA / 校准驱动。
 *
 * 串口命令：
 *   n <count>      统计次数（默认 2000）
 *   fsw <Hz>       设定 PWM 频率用于算占比
 *   marker 0|1     GPIO(IO_15) 标记开关
 *   bench          跑一次耗时统计
 *   read          打印一次原始对比
 */

#include <Arduino.h>
#include <v3plab.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/adc.h>
#include <esp_timer.h>
#endif

using namespace v3p;

static const uint32_t BAUD = 921600;

static PwmDriver    pwm;
static CurrentSense cs;
static Console      con;

static bool  markerOn = true;
static float fsw      = 20000.0f;
static int   benchN   = 2000;

static uint32_t usAnalog = 0, usIdf = 0, usMath = 0, usTotal = 0;

static void cmd_n(int argc, char** argv) { benchN = (int)Console::argi(argv, argc, 1, benchN); }

static void cmd_fsw(int argc, char** argv) {
  fsw = Console::argf(argv, argc, 1, fsw);
  pwm.setFrequency(fsw);
  Console::printfln("fsw = %.0f Hz  周期 = %.2f us", fsw, 1e6f / fsw);
}

static void cmd_marker(int argc, char** argv) {
  markerOn = Console::argi(argv, argc, 1, 1) != 0;
  Console::printfln("marker %s (IO_%d)", markerOn ? "ON" : "OFF", IO_15);
}

static void cmd_read(int, char**) {
#if defined(ARDUINO_ARCH_ESP32)
  const int64_t t0 = esp_timer_get_time();
  volatile int a1 = analogRead(M0_OUT1_CS);
  const int64_t t1 = esp_timer_get_time();
  volatile int a2 = adc1_get_raw(ADC1_CHANNEL_3);   // GPIO39
  const int64_t t2 = esp_timer_get_time();
  Console::printfln("analogRead = %d (%lld us)   adc1_get_raw = %d (%lld us)", (int)a1,
                    (long long)(t1 - t0), (int)a2, (long long)(t2 - t1));
#else
  Console::print("ESP32 only\r\n");
#endif
}

/// 统计：ADC / 算法 / 总计
static void cmd_bench(int, char**) {
#if defined(ARDUINO_ARCH_ESP32)
  uint32_t sumAnalog = 0, sumIdf = 0, sumMath = 0, sumTotal = 0;
  const int N = benchN > 0 ? benchN : 100;

  for (int i = 0; i < N; i++) {
    if (markerOn) digitalWrite(IO_15, HIGH);
    const int64_t t0 = esp_timer_get_time();

    // --- 采样 ---
    float ia, ib, ic;
    cs.read(ia, ib, ic);
    const int64_t t1 = esp_timer_get_time();

    // --- 算法：Clarke → Park → PI → 反Park → SVPWM ---
    const AlphaBeta iab = clarke(ia, ib);
    const DQ idq = park(iab, 0.3f);
    const AlphaBeta v = invPark(DQ{idq.d, idq.q}, 0.3f);
    Duty d = svpwm7(v, VDC_DEFAULT);
    clampDuty(d);
    pwm.setDuty(d);
    const int64_t t2 = esp_timer_get_time();

    // --- IDF 原始读（对比用）---
    volatile int r = adc1_get_raw(ADC1_CHANNEL_0);
    const int64_t t3 = esp_timer_get_time();
    (void)r;

    if (markerOn) digitalWrite(IO_15, LOW);

    sumAnalog += (uint32_t)(t1 - t0);
    sumMath   += (uint32_t)(t2 - t1);
    sumIdf    += (uint32_t)(t3 - t2);
    sumTotal  += (uint32_t)(t3 - t0);
    delayMicroseconds(50);
  }

  usAnalog = sumAnalog / N;
  usMath   = sumMath / N;
  usIdf    = sumIdf / N;
  usTotal  = sumTotal / N;
  const float period = 1e6f / fsw;

  Console::printfln("--- %d 次平均 ---", N);
  Console::printfln("采样 (analogRead x2 + 换算) = %u us", (unsigned)usAnalog);
  Console::printfln("算法 (Clarke/Park/PI/反Park/SVPWM) = %u us", (unsigned)usMath);
  Console::printfln("adc1_get_raw x1              = %u us", (unsigned)usIdf);
  Console::printfln("合计                          = %u us", (unsigned)usTotal);
  Console::printfln("PWM 周期 %.1f us → 占用 %.1f%%", period, 100.0f * (float)usTotal / period);
  Console::printfln("→ fsw 上限约 %.1f kHz（按 50%% 预算）", 0.5f * 1e6f / fmaxf(1.0f, (float)usTotal) / 1000.0f);
#else
  Console::print("ESP32 only\r\n");
#endif
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E2.4 ADC 采样时序与算法耗时 ===\r\n");

  pinMode(IO_15, OUTPUT);
  digitalWrite(IO_15, LOW);

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(fsw);
  pwm.setDuty(0.5f, 0.5f, 0.5f);
  // 本实验不需要驱动板输出（只测时序），保持失能

  cs.begin(M0_OUT1_CS, M0_OUT2_CS);
  cs.calibrate(512);

#if defined(ARDUINO_ARCH_ESP32)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);   // GPIO39
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);   // GPIO36
#endif

  con.add("n",      cmd_n,      "n <count>");
  con.add("fsw",    cmd_fsw,    "fsw <Hz>");
  con.add("marker", cmd_marker, "marker 0|1");
  con.add("bench",  cmd_bench,  "跑耗时统计");
  con.add("read",   cmd_read,   "打印一次 analogRead vs adc1_get_raw");

  telemetry::setChannels(4);
  telemetry::setChannelName(0, "us_sample");
  telemetry::setChannelName(1, "us_math");
  telemetry::setChannelName(2, "us_total");
  telemetry::setChannelName(3, "duty_pct");

  Console::print("命令：bench / read / n 2000 / fsw 20000\r\n");
  Console::print("示波器：CH1 接 IO_15(标记脉冲)，CH2 接 M0_GH1 或相电压，看采样发生在周期何处\r\n");
}

void loop() {
  con.poll();
  telemetry::set(0, (float)usAnalog);
  telemetry::set(1, (float)usMath);
  telemetry::set(2, (float)usTotal);
  telemetry::set(3, 100.0f * (float)usTotal / (1e6f / fsw));
  telemetry::tick();
}
