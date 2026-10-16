/**
 * E1.7 ★ 栅极驱动与自举实测（阶段 1）—— 与 DJI 真题 2 对应最直接的实验
 *
 * ★★ 重要更正（本实验的存在理由）：
 *   网上流传的"V3P 死区可由 MCU 设置、所以死区扫描实验可做"是【错的】。
 *   实际原理图（V3P驱动板最新版原理图.pdf）里 FD6287T 的
 *      HIN1~3 与 LIN1#~3# 被接在同一组网络（M0_IN1/2/3 各出现两次），
 *   也就是每相只有 1 根 MCU PWM 线 → 互补与死区完全由驱动 IC 内部产生：
 *      FD6287T 内部死区 DT = 100/200/300 ns (min/typ/max)，且有硬件互锁。
 *   MCU 无法设置死区；"死区电压补偿"仍然可以做（见 E1.6）。
 *
 * 本实验给出 4 种测试波形，配合示波器测量：
 *   mode 0  三相 50% 静态      → 看稳定栅极电平、自举电压
 *   mode 1  单相方波 f Hz      → 同时看 GH/GL，量出【真实死区】
 *   mode 2  单相 100% 占空比    → 自举电容持续放电 → 上管掉驱动（经典失效）
 *   mode 3  单脉冲             → 量 ton/toff/上升下降时间/振铃
 *
 * 测量要点（务必先读）：
 *   · 探头地线要短（用弹簧地针），长鳄鱼夹会引入假振铃
 *   · 测 VB–VS 必须差分探头（VS 是浮地的），或两通道相减
 *   · 第一次务必用 12 V / 0.3 A 限流
 *   · 板上栅极电阻 R26 等 = 2.2 Ω，栅源下拉 10 kΩ；自举电容 1 µF（C44/45/47）
 *
 * 串口命令：
 *   mode 0|1|2|3
 *   f <Hz>       方波频率（mode 1/2 用）
 *   duty <0~1>   方波占空比（mode 1）
 *   en 0|1       使能驱动板（mode 2 危险，会往电机灌直流）
 *   stop
 */

#include <Arduino.h>
#include <v3plab.h>

using namespace v3p;

static const uint32_t BAUD = 921600;

static PwmDriver    pwm;
static Console      con;

static int   mode  = 0;
static float fsw   = 1000.0f;    // mode 1/2 的方波频率（低到能用示波器抓）
static float dutyA = 0.5f;
static bool  run   = false;

static void cmd_mode(int argc, char** argv) {
  mode = (int)Console::argi(argv, argc, 1, mode);
  run  = false;
  pwm.setDuty(0.5f, 0.5f, 0.5f);
  switch (mode) {
    case 0: Console::print("mode 0: 三相 50% 静态（安全）\r\n"); break;
    case 1: Console::print("mode 1: A 相方波，B/C 相 50%（量死区）\r\n"); break;
    case 2: Console::print("mode 2: A 相 100%（自举放电测试）⚠ 会往电机灌直流，请限流 0.3A\r\n"); break;
    case 3: Console::print("mode 3: 单脉冲（量 ton/toff/振铃）\r\n"); break;
    default: Console::print("mode 0..3\r\n"); break;
  }
}

static void cmd_f(int argc, char** argv) {
  fsw = Console::argf(argv, argc, 1, fsw);
  // 用一个专用 LEDC 频率（直接改 PWM 频率）
  pwm.setFrequency(fsw);
  Console::printfln("fsw = %.1f Hz", fsw);
}

static void cmd_duty(int argc, char** argv) {
  dutyA = Console::argf(argv, argc, 1, dutyA);
  Console::printfln("dutyA = %.3f", dutyA);
}

static void cmd_en(int argc, char** argv) {
  const bool on = Console::argi(argv, argc, 1, 0) != 0;
  pwm.enable(on);
  run = on;
  Console::printfln("driver %s", on ? "ENABLED" : "disabled");
}

static void cmd_stop(int, char**) {
  run = false;
  pwm.coast();
  Console::print("stopped\r\n");
}

void setup() {
  telemetry::begin(BAUD);
  con.begin(BAUD);
  delay(300);

  Console::print("\r\n=== E1.7 栅极驱动与自举实测 ===\r\n");
  Console::print("★ 死区由 FD6287T 内部固定（typ 200 ns），MCU 无法设置。\r\n");
  Console::print("★ 测相电压/VB-VS 必须差分探头；测栅极请用弹簧地针。\r\n");

  pwm.begin(M0_IN1, M0_IN2, M0_IN3, MOTOR_EN, 0);
  pwm.setFrequency(fsw);
  pwm.setDuty(0.5f, 0.5f, 0.5f);

  con.add("mode", cmd_mode, "mode 0|1|2|3");
  con.add("f",    cmd_f,    "f <Hz> 方波频率");
  con.add("duty", cmd_duty, "duty <0~1>");
  con.add("en",   cmd_en,   "en 0|1");
  con.add("stop", cmd_stop, "停止并失能");

  telemetry::setChannels(3);
  telemetry::setChannelName(0, "duty_a");
  telemetry::setChannelName(1, "mode");
  telemetry::setChannelName(2, "fsw");

  Console::print("先 mode 0，再 en 1；用示波器看 M0_GH1 / M0_GL1 与 VB1-VS1。\r\n");
}

void loop() {
  con.poll();

  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  const float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;

  if (run) {
    if (mode == 1) {
      // A 相方波，B/C 保持 50%
      static float ph = 0.0f;
      ph += dt * fsw;
      if (ph >= 1.0f) ph -= 1.0f;
      const float a = (ph < dutyA) ? 1.0f : 0.0f;
      pwm.setDuty(a, 0.5f, 0.5f);
    } else if (mode == 2) {
      // A 相 100%：上管常通，自举电容持续放电
      pwm.setDuty(1.0f, 0.5f, 0.5f);
    } else if (mode == 3) {
      // 单脉冲：每 10 ms 发一个 20 µs 高电平，便于示波器单次触发
      static float ph = 0.0f;
      ph += dt / 0.010f;
      if (ph >= 1.0f) ph -= 1.0f;
      const float a = (ph < 0.002f) ? 1.0f : 0.0f;
      pwm.setDuty(a, 0.5f, 0.5f);
    } else {
      pwm.setDuty(0.5f, 0.5f, 0.5f);
    }
  }

  telemetry::set(0, mode == 2 ? 1.0f : dutyA);
  telemetry::set(1, (float)mode);
  telemetry::set(2, pwm.frequency());
  telemetry::tick();
}
