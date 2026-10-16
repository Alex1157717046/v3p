/**
 * E7.1 STM32F103 + Q15 定点 FOC —— 浮点 vs 定点对比固件
 *
 * 这是阶段 7 的实操代码。它把同一套 FOC 算法用【浮点】和【Q15 定点】各实现一遍，
 * 在同一块 STM32F103（Cortex-M3，无 FPU）上跑，用 DWT 周期计数器直接量出两者的
 * 单次迭代周期数 —— 这就是 E7.1 要的那张对比表。
 *
 * ★ 引脚映射（来自 docs/STM32转接板引脚映射对照表.md，是从原理图逆向出来的）：
 *     M0_IN1/2/3 → PA0/PA1/PA2 = TIM2_CH1/2/3    （✅ 同一个定时器）
 *     M0_OUT1/CS → PB0 = ADC1_IN8
 *     M0_OUT2_CS → PB1 = ADC1_IN9
 *     MOTOR_EN   → PA8   ⚠⚠ 转接板上网名写成了 A8，与 PA8 未连通！
 *                          用之前请先用万用表量 S2.5 ↔ E2.12 是否真的不通
 *     SCL_0/SDA_0→ PB10/PB11 = I2C2（本固件用开环角度，不读编码器，所以不用）
 *     TXD/RXD    → PA9/PA10 = USART1
 *
 * ⚠ 另一个已知缺陷：M1 的三路 PWM 跨 TIM2/TIM3（PA3+PA6+PA7），本固件只做 M0。
 *
 * 串口命令（115200）：
 *   f | q        切换 浮点 / Q15 定点
 *   b <n>        跑 n 次基准测试，打印周期数与耗时
 *   cmp          浮点与定点各跑一遍并打印对比表
 *   v <V>        开环电压幅值（标幺，0~1）
 *   f1 <Hz>      开环电频率
 *   en 0|1       使能驱动板
 *   cp           打印电流环状态
 */

#include <Arduino.h>
#include "foc_q15.h"

using namespace focq15;

// ============================ 引脚 ============================
static const int PIN_PWM_A = PA0;
static const int PIN_PWM_B = PA1;
static const int PIN_PWM_C = PA2;
static const int PIN_EN    = PA8;    // ⚠ 见文件头说明
static const int PIN_CS1   = PB0;
static const int PIN_CS2   = PB1;
static const int PIN_MARK  = PB12;   // WCET 标记脚（示波器用）

// ============================ 标度（★ 定点化最关键的一步）============================
// 电流基准：2 A ↔ Q15 满量程 1.0
static const float I_BASE = 2.0f;
// 电压基准：Vdc/√3 ↔ 1.0（与 foc_q15.h 的 svpwm 约定一致）
static const float VDC = 12.0f;
static const float V_BASE = VDC / 1.7320508f;
// 电流采样：INA240A2 ×50 + 0.01 Ω，零点 1.65 V
static const float V_PER_A = 0.5f;
static const float ADC_VREF = 3.3f;

// ============================ 状态 ============================
static bool  g_useQ15 = true;
static bool  g_enabled = false;
static float g_vMag = 0.30f;        // 开环电压（标幺）
static float g_fElec = 5.0f;        // 开环电频率 Hz
static float g_theta = 0.0f;

// 浮点侧状态
static float fIntegD = 0, fIntegQ = 0;
// 定点侧状态
static PiQ15 qPidD, qPidQ;
static uint16_t qTheta = 0;

static uint32_t g_cycFloat = 0, g_cycQ15 = 0;

// ============================ DWT 周期计数器 ============================
static void dwtInit() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt() { return DWT->CYCCNT; }

// ============================ PWM（TIM2 三通道）============================
static HardwareTimer* g_tim = nullptr;

static void pwmInit() {
  g_tim = new HardwareTimer(TIM2);
  // 20 kHz，初始 50%
  g_tim->setPWM(1, PIN_PWM_A, 20000, 50.0f);
  g_tim->setPWM(2, PIN_PWM_B, 20000, 50.0f);
  g_tim->setPWM(3, PIN_PWM_C, 20000, 50.0f);
}
static void setDuty(float a, float b, float c) {
  // duty 0~1 → 百分比，并钳位
  a = a < 0 ? 0 : (a > 1 ? 1 : a);
  b = b < 0 ? 0 : (b > 1 ? 1 : b);
  c = c < 0 ? 0 : (c > 1 ? 1 : c);
  g_tim->setCaptureCompare(1, a * 100.0f, PERCENT_COMPARE_FORMAT);
  g_tim->setCaptureCompare(2, b * 100.0f, PERCENT_COMPARE_FORMAT);
  g_tim->setCaptureCompare(3, c * 100.0f, PERCENT_COMPARE_FORMAT);
}

// ============================ 电流采样 ============================
static float g_off1 = 1.65f, g_off2 = 1.65f;

static void csCalibrate() {
  analogReadResolution(12);
  double s1 = 0, s2 = 0;
  for (int i = 0; i < 512; i++) {
    s1 += (float)analogRead(PIN_CS1) * ADC_VREF / 4095.0f;
    s2 += (float)analogRead(PIN_CS2) * ADC_VREF / 4095.0f;
    delayMicroseconds(50);
  }
  g_off1 = (float)(s1 / 512);
  g_off2 = (float)(s2 / 512);
}

// ============================ 浮点 FOC ============================
static const float PI_F = 3.14159265358979f;
static const float SQRT3_F = 1.7320508075688772f;
static const float TWO_PI_F_LOCAL = 6.283185307179586f;
// Kp = ωc·L = 2π·300·0.01 ≈ 18.85 V/A → 换算到标幺
static float g_kpPu = 0.0f, g_kiPu = 0.0f;

static void runFloatFoc(float ia, float ib, float theta, float dt, float* da, float* db, float* dc) {
  // Clarke
  const float al = ia;
  const float be = (ia + 2.0f * ib) / SQRT3_F;
  // Park
  const float c = cosf(theta), s = sinf(theta);
  const float id = al * c + be * s;
  const float iq = -al * s + be * c;
  // PI（电流给定 id*=0, iq*=0.2 A 标幺）
  const float idRef = 0.0f, iqRef = 0.2f / I_BASE;
  const float ep = idRef - id, eq = iqRef - iq;
  float vd = g_kpPu * ep + fIntegD;
  float vq = g_kpPu * eq + fIntegQ;
  fIntegD += g_kiPu * ep * dt;
  fIntegQ += g_kiPu * eq * dt;
  // 限幅
  const float vlim = 0.95f;
  if (vd > vlim) vd = vlim; if (vd < -vlim) vd = -vlim;
  if (vq > vlim) vq = vlim; if (vq < -vlim) vq = -vlim;
  fIntegD = vd; fIntegQ = vq;      // 简化：积分器限幅
  // 反 Park
  const float va = vd * c - vq * s;
  const float vb = vd * s + vq * c;
  // SVPWM（min-max）
  const float vA = va, vB = -0.5f * va + 0.5f * SQRT3_F * vb, vC = -0.5f * va - 0.5f * SQRT3_F * vb;
  const float vmax = vA > vB ? (vA > vC ? vA : vC) : (vB > vC ? vB : vC);
  const float vmin = vA < vB ? (vA < vC ? vA : vC) : (vB < vC ? vB : vC);
  const float off = -0.5f * (vmax + vmin);
  *da = 0.5f + (vA + off) / SQRT3_F;
  *db = 0.5f + (vB + off) / SQRT3_F;
  *dc = 0.5f + (vC + off) / SQRT3_F;
}

// ============================ Q15 定点 FOC ============================
static void runQ15Foc(q15_t ia, q15_t ib, uint16_t theta, uint32_t dt_us,
                      uint16_t* da, uint16_t* db, uint16_t* dc) {
  q15_t al, be, id, iq;
  clarke(ia, ib, &al, &be);
  park(al, be, theta, &id, &iq);

  const q15_t idRef = 0;
  const q15_t iqRef = fromFloat(0.2f / I_BASE);      // ★ 标度统一：与浮点同一个基准
  const q15_t vd = qPidD.update((q15_t)(idRef - id), dt_us);
  const q15_t vq = qPidQ.update((q15_t)(iqRef - iq), dt_us);

  q15_t va, vb;
  invPark(vd, vq, theta, &va, &vb);
  svpwm(va, vb, da, db, dc);
}

// ============================ 基准测试 ============================
static void bench(uint32_t n, bool q15) {
  // 造一个固定的"采样值"，避免 ADC 抖动影响计时
  const float iaF = 0.35f, ibF = -0.18f;
  const q15_t iaQ = fromFloat(iaF / I_BASE), ibQ = fromFloat(ibF / I_BASE);

  digitalWrite(PIN_MARK, HIGH);
  const uint32_t t0 = dwt();
  for (uint32_t k = 0; k < n; k++) {
    g_theta += 0.001f;
    if (g_theta > TWO_PI_F_LOCAL) g_theta -= TWO_PI_F_LOCAL;
    float da, db, dc;
    if (q15) {
      qTheta = (uint16_t)((uint32_t)(g_theta / TWO_PI_F_LOCAL * 65536.0f) & 0xFFFF);
      uint16_t ua, ub, uc;
      runQ15Foc(iaQ, ibQ, qTheta, 200, &ua, &ub, &uc);
      da = 0; db = 0; dc = 0;
      (void)da; (void)db; (void)dc; (void)ua; (void)ub; (void)uc;
    } else {
      runFloatFoc(iaF, ibF, g_theta, 0.0002f, &da, &db, &dc);
      (void)da; (void)db; (void)dc;
    }
  }
  const uint32_t dt = dwt() - t0;
  digitalWrite(PIN_MARK, LOW);

  const float perIter = (float)dt / (float)n;
  const float us = perIter / 72.0f;            // 72 MHz → 1 cycle = 1/72 µs
  if (q15) g_cycQ15 = (uint32_t)perIter; else g_cycFloat = (uint32_t)perIter;

  Serial.print(q15 ? "Q15 定点: " : "浮点    : ");
  Serial.print(perIter, 1);
  Serial.print(" cycles/次,  ");
  Serial.print(us, 2);
  Serial.print(" us/次,  ");
  Serial.print(1000000.0f / us, 0);
  Serial.println(" Hz 上限(单次迭代预算)");
}



// ============================ E2.6 双 MCU ADC 对比工具 ============================
// 用法：把同一路模拟信号（0.5~2.5 V）同时接到
//   ESP32 的 M0_OUT1_CS(GPIO39) 与 STM32 的 PB0(ADC1_IN8)
// 两边各跑本命令，比较：噪声 RMS、极值、直方图、以及 PWM 开/关时的干扰差异。
// ESP32 侧用 exp09_current 的 stat / exp11_adctiming 的 bench。
static void cmdAdcBench(bool pwmOn) {
  if (!pwmOn) {
    // 关掉 PWM 输出（三相 50% = 零电压）以便测"无开关干扰"的本底
    setDuty(0.5f, 0.5f, 0.5f);
  }
  const int N = 4096;
  uint32_t sum = 0, sum2 = 0;
  uint16_t mn = 4095, mx = 0;
  int hist[16] = {0};
  uint32_t t0 = micros();
  for (int i = 0; i < N; i++) {
    const uint16_t v = (uint16_t)analogRead(PIN_CS1);
    sum += v; sum2 += (uint32_t)v * v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    hist[v >> 8]++;
  }
  const uint32_t dt = micros() - t0;
  const double mean = (double)sum / N;
  const double var = (double)sum2 / N - mean * mean;
  const double std_ = var > 0 ? sqrt(var) : 0;

  Serial.println("---- STM32F103 ADC 质量测量 (PB0 / ADC1_IN8) ----");
  Serial.print("PWM 状态      : "); Serial.println(pwmOn ? "开（有开关干扰）" : "关（本底）");
  Serial.print("样本数        : "); Serial.println(N);
  Serial.print("均值(LSB)     : "); Serial.println(mean, 1);
  Serial.print("噪声 std(LSB) : "); Serial.println(std_, 2);
  Serial.print("噪声 std(mV)  : "); Serial.println(std_ * 3300.0f / 4095.0f, 2);
  Serial.print("最小/最大(LSB): "); Serial.print(mn); Serial.print(" / "); Serial.println(mx);
  Serial.print("峰峰值(LSB)   : "); Serial.println(mx - mn);
  Serial.print("采样率        : "); Serial.print(N * 1000000.0f / dt, 0); Serial.println(" Hz");
  Serial.print("直方图(每 256LSB): ");
  for (int i = 0; i < 16; i++) { Serial.print(hist[i]); Serial.print(i < 15 ? "," : "\n"); }
  Serial.println("→ 与 ESP32 的 exp09/exp11 数据对比，就是 E2.6 要的那张表");
  Serial.println("→ 注意两块 MCU 的参考电压不同：ESP32 约 1.1V 内部分压+衰减，STM32 是 VDDA=3.3V");
}

// ============================ 命令 ============================
static void cmdCompare() {
  Serial.println("---- E7.1 浮点 vs Q15 定点（STM32F103 @72 MHz）----");
  bench(2000, false);
  bench(2000, true);
  if (g_cycFloat && g_cycQ15) {
    Serial.print("加速比 = ");
    Serial.print((float)g_cycFloat / (float)g_cycQ15, 2);
    Serial.println(" ×");
    Serial.print("按 50% 时间预算，PWM 频率上限：浮点 ");
    Serial.print(72e6f / (2.0f * g_cycFloat * 1000.0f), 1);
    Serial.print(" kHz，定点 ");
    Serial.print(72e6f / (2.0f * g_cycQ15 * 1000.0f), 1);
    Serial.println(" kHz");
  }
  Serial.println("★ 这就是 E7.1 要的那张对比表（把数字填进你的实验报告）");
}

static void handleLine(char* line) {
  char* cmd = strtok(line, " \r\n");
  if (!cmd) return;
  if (!strcmp(cmd, "f")) { g_useQ15 = false; Serial.println("浮点模式"); }
  else if (!strcmp(cmd, "q")) { g_useQ15 = true; Serial.println("Q15 定点模式"); }
  else if (!strcmp(cmd, "cmp")) cmdCompare();
  else if (!strcmp(cmd, "adc")) cmdAdcBench(false);
  else if (!strcmp(cmd, "adcpwm")) cmdAdcBench(true);
  else if (!strcmp(cmd, "b")) { char* a = strtok(nullptr, " \r\n"); bench(a ? atoi(a) : 1000, g_useQ15); }
  else if (!strcmp(cmd, "v")) { char* a = strtok(nullptr, " \r\n"); if (a) g_vMag = atof(a); }
  else if (!strcmp(cmd, "f1")) { char* a = strtok(nullptr, " \r\n"); if (a) g_fElec = atof(a); }
  else if (!strcmp(cmd, "en")) {
    char* a = strtok(nullptr, " \r\n");
    g_enabled = a && atoi(a) != 0;
    digitalWrite(PIN_EN, g_enabled ? HIGH : LOW);
    Serial.println(g_enabled ? "driver ENABLED" : "driver disabled");
  } else if (!strcmp(cmd, "cp")) {
    Serial.print("mode="); Serial.print(g_useQ15 ? "Q15" : "float");
    Serial.print("  theta="); Serial.print(g_theta, 3);
    Serial.print("  vMag="); Serial.print(g_vMag, 3);
    Serial.print("  off1="); Serial.print(g_off1, 4);
    Serial.print("  off2="); Serial.println(g_off2, 4);
  } else {
    Serial.println("命令: f|q|cmp|b <n>|v <pu>|f1 <Hz>|en 0|1|cp|adc|adcpwm");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\r\n=== E7.1 STM32F103 Q15 定点 FOC ===");
  Serial.println("⚠ 转接板上 MOTOR_EN(PA8) 的网名被写成 A8，可能未连通 —— 先万用表确认");
  Serial.println("⚠ M1 三路 PWM 跨 TIM2/TIM3，本固件只做 M0");

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);
  pinMode(PIN_MARK, OUTPUT);
  digitalWrite(PIN_MARK, LOW);

  pwmInit();
  dwtInit();
  csCalibrate();

  // 电流环增益：Kp = ωc·L，换算成标幺（V_base/I_base）
  const float wc = 2.0f * PI_F * 300.0f;
  const float kpVperA = wc * 0.010f;          // V/A
  const float kiVperAs = wc * 10.0f;
  g_kpPu = kpVperA * I_BASE / V_BASE;
  g_kiPu = kiVperAs * I_BASE / V_BASE;
  qPidD.kp = qPidQ.kp = fromFloat(g_kpPu);
  qPidD.ki = qPidQ.ki = fromFloat(g_kiPu);
  qPidD.outMax = qPidQ.outMax = fromFloat(0.95f);
  qPidD.outMin = qPidQ.outMin = fromFloat(-0.95f);

  Serial.print("Kp(标幺) = "); Serial.print(g_kpPu, 4);
  Serial.print("  Ki(标幺) = "); Serial.println(g_kiPu, 4);
  Serial.println("输入 cmp 看浮点/定点对比；输入 adc / adcpwm 做 E2.6 的 ADC 质量测量");
}

void loop() {
  // 串口命令行
  static char buf[64];
  static int len = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < 63) {
      buf[len++] = c;
    }
  }

  // 20 kHz 环：这里用 millis 粗略限速演示；量产应放定时器中断
  static uint32_t last = 0;
  const uint32_t now = micros();
  if (now - last < 200) return;      // 5 kHz 演示用
  last = now;

  g_theta += 2.0f * PI_F * g_fElec * 0.0002f;
  if (g_theta > 2.0f * PI_F) g_theta -= 2.0f * PI_F;

  // 采样
  const float iaF = ((float)analogRead(PIN_CS1) * ADC_VREF / 4095.0f - g_off1) / V_PER_A;
  const float ibF = ((float)analogRead(PIN_CS2) * ADC_VREF / 4095.0f - g_off2) / V_PER_A;

  digitalWrite(PIN_MARK, HIGH);
  float da, db, dc;
  if (g_useQ15) {
    const q15_t iaQ = fromFloat(iaF / I_BASE);
    const q15_t ibQ = fromFloat(ibF / I_BASE);
    qTheta = (uint16_t)((uint32_t)(g_theta / (2.0f * PI_F) * 65536.0f) & 0xFFFF);
    uint16_t ua, ub, uc;
    runQ15Foc(iaQ, ibQ, qTheta, 200, &ua, &ub, &uc);
    da = (float)ua / 65535.0f;
    db = (float)ub / 65535.0f;
    dc = (float)uc / 65535.0f;
  } else {
    runFloatFoc(iaF, ibF, g_theta, 0.0002f, &da, &db, &dc);
  }
  setDuty(da, db, dc);
  digitalWrite(PIN_MARK, LOW);
}
