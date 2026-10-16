/**
 * E2.2 电流采样标定（阶段 2）
 *
 * 目标：让"ADC 码值 → 安培"这条换算准确。
 * 验收：零偏 < 10 mA；增益误差 < 3%；给出实测分辨率。
 *
 * 硬件依据（原理图核对）：
 *   INA240A2（增益 50 V/V）×4，分流 0.01 Ω（丝印 R010），REF 偏置 1.65 V
 *   → 灵敏度 0.5 V/A，量程 ±3.3 A
 *   ADC：ESP32 ADC1，GPIO39/36（M0）、35/34（M1）
 *   ⚠ ESP32 ADC 非线性可达 ±6%（INL ±12 LSB），所以"多点标定"比单点斜率更靠谱
 *
 * ★ 反推法（推荐，一次性同时得到 R·G）：
 *   给电机通一个已知直流电流（堵转，用外部电流表读真值），
 *   读 INA240 输出增量 → R·G = ΔV/I。G=50 已知 ⇒ 反推 R_shunt。
 *   命令：known <I>   在施加已知电流时敲一下
 *
 * 串口命令：
 *   cal             零偏标定（必须无电流）
 *   gain <V/A>      设灵敏度
 *   sign 1|-1|2 -1  分别设两路方向
 *   stat <sec>      噪声/分辨率统计
 *   known <A>       反推 R·G（先让电流稳定）
 *   read            打印一次原始读数
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static CurrentSense cs0, cs1;
static Console      con;

static float vPerA = CSA_V_PER_A;

static void cmd_cal(int, char**) {
  Console::print("calibrating (ZERO current required)...\r\n");
  cs0.calibrate(2048);
  cs1.calibrate(2048);
  Console::printfln("M0 offset: %.4f / %.4f V", cs0.offset1(), cs0.offset2());
  Console::printfln("M1 offset: %.4f / %.4f V", cs1.offset1(), cs1.offset2());
}

static void cmd_gain(int argc, char** argv) {
  vPerA = Console::argf(argv, argc, 1, vPerA);
  Console::printfln("v_per_a = %.4f V/A  (量程 ±%.2f A)", vPerA, 1.65f / vPerA);
}

static void cmd_sign(int argc, char** argv) {
  const int which = (int)Console::argi(argv, argc, 1, 1);
  const float s   = Console::argf(argv, argc, 2, 1.0f);
  if (which == 1) cs0.sign1 = (s >= 0) ? 1.0f : -1.0f;
  else            cs0.sign2 = (s >= 0) ? 1.0f : -1.0f;
  Console::printfln("sign%d = %.0f", which, which == 1 ? cs0.sign1 : cs0.sign2);
}

static void cmd_read(int, char**) {
  uint16_t r1, r2;
  float v1, v2, i1, i2, ic;
  cs0.readRaw(r1, r2);
  cs0.readVolts(v1, v2);
  cs0.readTwo(i1, i2);
  ic = -(i1 + i2);
  Console::printfln("raw = %4u %4u   V = %.4f %.4f   I = %+.4f %+.4f  (ic=%+.4f) A", r1, r2, v1,
                    v2, i1, i2, ic);
  Console::printfln("offset = %.4f %.4f V   v_per_a = %.4f", cs0.offset1(), cs0.offset2(), vPerA);
}

static void cmd_stat(int argc, char** argv) {
  const float sec = Console::argf(argv, argc, 1, 3.0f);
  const uint32_t t0 = millis();
  const int N = 4000;
  double s = 0, s2 = 0, sr = 0, sr2 = 0;
  int n = 0;
  while (n < N && millis() - t0 < (uint32_t)(sec * 1000.0f)) {
    uint16_t r1, r2;
    cs0.readRaw(r1, r2);
    float i1, i2;
    cs0.readTwo(i1, i2);
    s += i1; s2 += (double)i1 * i1;
    sr += r1; sr2 += (double)r1 * r1;
    n++;
    delayMicroseconds(100);
  }
  const double m = s / n, v = s2 / n - m * m;
  const double mr = sr / n, vr = sr2 / n - mr * mr;
  const float stdA  = (float)sqrt(v > 0 ? v : 0);
  const float stdR  = (float)sqrt(vr > 0 ? vr : 0);
  Console::printfln("n=%d  I std = %.4f A (%.1f mA)   raw std = %.2f LSB", n, stdA, stdA * 1000.0f, stdR);
  Console::printfln("理论 1 LSB = %.2f mA", 1000.0f * (ADC_VREF / 4095.0f) / vPerA);
  Console::printfln("mean I = %.4f A (零偏残差，应≈0)", (float)m);
}

static void cmd_known(int argc, char** argv) {
  const float iknown = Console::argf(argv, argc, 1, 0.0f);
  if (fabsf(iknown) < 1e-4f) { Console::print("用法：known <已知电流A>\r\n"); return; }
  const float v1 = cs0.rawToVolt((uint16_t)analogRead(M0_OUT1_CS)) - cs0.offset1();
  const float v2 = cs0.rawToVolt((uint16_t)analogRead(M0_OUT2_CS)) - cs0.offset2();
  Console::printfln("ΔV1 = %.5f V  ΔV2 = %.5f V", v1, v2);
  const float v = (fabsf(v1) > fabsf(v2)) ? v1 : v2;
  const float rg = v / iknown;               // V/A = R_shunt × G
  Console::printfln("R×G = %.5f V/A  → 若 G=50，则 R_shunt = %.5f ohm", rg, rg / INA_GAIN);
  Console::printfln("（出厂 0.01 Ω 应得 0.5 V/A；若为 0.001 Ω 则 0.05 V/A）");
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);
  Console::print("\r\n=== E2.2 电流采样标定 ===\r\n");

  cs0.begin(M0_OUT1_CS, M0_OUT2_CS, vPerA);
  cs1.begin(M1_OUT1_CS, M1_OUT2_CS, vPerA);
  cs0.calibrate(2048);
  cs1.calibrate(2048);

  Console::printfln("offset M0 = %.4f / %.4f V", cs0.offset1(), cs0.offset2());

  con.add("cal",   cmd_cal,   "零偏标定（无电流）");
  con.add("gain",  cmd_gain,  "gain <V/A>");
  con.add("sign",  cmd_sign,  "sign <1|2> <1|-1>");
  con.add("stat",  cmd_stat,  "stat <sec>");
  con.add("known", cmd_known, "known <A> 反推 R×G");
  con.add("read",  cmd_read,  "打印一次读数");

  telemetry::setChannels(6);
  telemetry::setChannelName(0, "i_a");
  telemetry::setChannelName(1, "i_b");
  telemetry::setChannelName(2, "v_a");
  telemetry::setChannelName(3, "v_b");
  telemetry::setChannelName(4, "offset_a");
  telemetry::setChannelName(5, "offset_b");

  Console::print("命令：cal / read / stat 3 / known 1.0\r\n");
}

void loop() {
  con.poll();

  float ia, ib, ic;
  cs0.read(ia, ib, ic);

  telemetry::set(0, ia);
  telemetry::set(1, ib);
  telemetry::set(2, cs0.rawToVolt((uint16_t)analogRead(M0_OUT1_CS)));
  telemetry::set(3, cs0.rawToVolt((uint16_t)analogRead(M0_OUT2_CS)));
  telemetry::set(4, cs0.offset1());
  telemetry::set(5, cs0.offset2());
  telemetry::tick();
}
