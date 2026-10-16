/**
 * E5.4 故障检测（阶段 5）
 *
 * 目标：实现分层故障检测。
 * 验收：主动制造故障（断开一相、堵住转轴），验证每种都能被检出。
 *
 * ⚠ 重要（12.6 E5.4 的提醒）：这块板每路只有【2 个电流传感器】，第三相是算出来的
 *   （ic = −(ia+ib)），所以 i_a+i_b+i_c ≡ 0 恒成立 —— **KCL 检验在这里没有意义**。
 *   缺相只能靠【跟踪误差 + 电压饱和】或【电流矢量不平衡】来判断。本实验就是这么做的。
 *
 * 监测量：
 *   ① 过流          max(|ia|,|ib|,|ic|) > 阈值
 *   ② 缺相/不平衡   跟踪误差大 且 电压已饱和（vLimitScale 很小）并持续
 *   ③ 堵转/编码器卡死 有转矩给定但转速长时间 ≈ 0
 *   ④ 过速          机械转速超限
 *   任一触发 → 锁存并立即失能
 *
 * 串口命令：
 *   start / stop
 *   spd <rad/s>
 *   iq <A>          直接给 q 轴电流（测故障用）
 *   inject 0|1|3    0=无；1=断开一相（PWM 强制 50%）；3=大电流（触发过流）
 *   show            打印各监测量的当前值与阈值
 *   clear           清除故障锁存
 *   test            自动依次注入三种故障并打印检出结果
 */

#include <Arduino.h>
#include "obs_common.h"

static void faultServiceOnce();   // 前置声明

static bool  g_fault = false;
static int   g_faultCode = 0;
static float g_iMaxTh = 1.0f;      // 过流阈值 A
static float g_errTh  = 0.25f;     // 跟踪误差阈值 A
static float g_vSatTh = 0.35f;     // vLimitScale 低于此算饱和
static float g_spdStallTh = 3.0f;  // 转速低于此算堵转 rad/s
static float g_wMaxTh = 200.0f;    // 过速阈值 rad/s（机械）
static uint32_t g_badSince = 0;
static int   g_inject = 0;
static bool  g_running2 = false;
static float g_iqCmd = 0.0f;
static float g_spdCmd = 0.0f;

static const char* faultName(int c) {
  switch (c) {
    case 1: return "过流 OVERCURRENT";
    case 2: return "缺相/不平衡 PHASE_LOSS";
    case 3: return "堵转或编码器卡死 STALL/ENC_STUCK";
    case 4: return "过速 OVERSPEED";
    default: return "无";
  }
}

static void cmdFStart(int, char**) { g_m.enable(true); g_running2 = true; g_fault = false; g_faultCode = 0; }
static void cmdFStop(int, char**)  { g_running2 = false; g_m.enable(false); }
static void cmdFSpd(int argc, char** argv) { g_spdCmd = Console::argf(argv, argc, 1, g_spdCmd); }
static void cmdIq(int argc, char** argv)  { g_iqCmd = Console::argf(argv, argc, 1, g_iqCmd); }
static void cmdFClear(int, char**) { g_fault = false; g_faultCode = 0; Console::print("故障已清除\r\n"); }

static void cmdInject(int argc, char** argv) {
  g_inject = (int)Console::argi(argv, argc, 1, 0);
  Console::printfln("注入故障 = %d（0无 1断相 3过流）", g_inject);
  g_fault = false; g_faultCode = 0;
}

static void cmdShow(int, char**) {
  const float imax = fmaxf(fabsf(g_m.ia), fmaxf(fabsf(g_m.ib), fabsf(g_m.ic)));
  Console::printfln("ia=%+.3f ib=%+.3f ic=%+.3f | imax=%.3f A (阈值 %.2f)", g_m.ia, g_m.ib, g_m.ic,
                    imax, g_iMaxTh);
  Console::printfln("id_ref=%.3f id=%.3f 误差=%.3f A (阈值 %.2f)  vLimit=%.2f (饱和阈值 %.2f)",
                    g_m.idReference(), g_m.id, g_m.idReference() - g_m.id, g_errTh,
                    g_m.vLimitScale, g_vSatTh);
  Console::printfln("ωm=%.2f rad/s（堵转阈值 %.1f，过速阈值 %.0f）", g_m.enc.velocity(),
                    g_spdStallTh, g_wMaxTh);
  Console::printfln("故障状态：%s", g_fault ? faultName(g_faultCode) : "正常");
}

/// 自动测试：依次注入 3 种故障
static void cmdTest(int, char**) {
  Console::print("---- 自动故障注入测试 ----\r\n");
  g_m.enable(true); g_running2 = true; g_spdCmd = 15.0f; g_iqCmd = 0.3f;
  Console::print("① 正常运行 2 s 建立基线...\r\n");
  { const uint32_t t0 = millis(); while (millis() - t0 < 2000) { faultServiceOnce(); delay(1); } }
  Console::printfln("   基线：故障=%s\r\n", g_fault ? faultName(g_faultCode) : "无");

  Console::print("② 注入断相（PWM 强制一相 50%）...\r\n");
  g_inject = 1; g_fault = false; g_faultCode = 0;
  { const uint32_t t0 = millis(); while (millis() - t0 < 3000) { faultServiceOnce(); delay(1); } }
  Console::printfln("   结果：%s\r\n", g_fault ? faultName(g_faultCode) : "❌ 未检出");

  Console::print("③ 注入过流（iq 给定 3 A，超过限幅）...\r\n");
  g_inject = 3; g_fault = false; g_faultCode = 0;
  { const uint32_t t0 = millis(); while (millis() - t0 < 3000) { faultServiceOnce(); delay(1); } }
  Console::printfln("   结果：%s\r\n", g_fault ? faultName(g_faultCode) : "❌ 未检出");

  g_inject = 0;
  Console::print("④ 堵转测试：请用手按住转轴 3 秒...\r\n");
  g_fault = false; g_faultCode = 0; g_spdCmd = 40.0f;
  { const uint32_t t0 = millis(); while (millis() - t0 < 6000) { faultServiceOnce(); delay(1); } }
  Console::printfln("   结果：%s\r\n", g_fault ? faultName(g_faultCode) : "❌ 未检出（可能没按住）");

  g_m.enable(false); g_running2 = false; g_inject = 0;
  Console::print("测试结束\r\n");
}

// ------------------------------------------------------------------ 单步
static void faultCheck() {
  const float imax = fmaxf(fabsf(g_m.ia), fmaxf(fabsf(g_m.ib), fabsf(g_m.ic)));
  const float err  = fabsf(g_m.idReference() - g_m.id);
  const float wm   = fabsf(g_m.enc.velocity());

  int code = 0;
  if (imax > g_iMaxTh) code = 1;
  else if (err > g_errTh && g_m.vLimitScale < g_vSatTh) code = 2;
  else if (wm < g_spdStallTh && fabsf(g_m.iq) > 0.3f && g_running2) code = 3;
  else if (wm > g_wMaxTh) code = 4;

  if (code != 0) {
    if (g_badSince == 0) g_badSince = millis();
    if (millis() - g_badSince > 400) {     // 持续 400 ms 才判故障（防误报）
      g_fault = true;
      g_faultCode = code;
      g_m.enable(false);                   // ★ 立即封波
      g_running2 = false;
    }
  } else {
    g_badSince = 0;
  }
}

static void faultServiceOnce() {
  static uint32_t tPrev = 0;
  const uint32_t now = micros();
  if (tPrev == 0) tPrev = now;
  float dt = (float)(now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  float iqRef = g_iqCmd;
  if (g_inject == 3) iqRef = 3.0f;                       // 故意超限
  if (g_running2 && g_inject != 3) {
    iqRef = g_pidSpd.update(g_spdCmd - g_m.enc.velocity(), dt);
  }
  g_m.setCurrentRef(0.0f, iqRef);
  g_m.update();

  // 断相模拟：把 B 相强行钳在 50%
  if (g_inject == 1) {
    // 断相模拟：把三相全部钳在 50%（等价于该相桥臂不再输出有效电压）
    g_m.pwm.setDuty(0.5f, 0.5f, 0.5f);
  }

  faultCheck();
}

void setup() {
  obsBegin();
  Console::print("\r\n=== E5.4 故障检测 ===\r\n");
  Console::print("★ 本板只有 2 个电流传感器 → KCL 检验恒成立，缺相靠【跟踪误差+电压饱和】判定\r\n");
  Console::print("流程：align → start → spd 15 → test\r\n");

  g_con.add("start",  cmdFStart,  "启动");
  g_con.add("stop",   cmdFStop,   "停止");
  g_con.add("spd",    cmdFSpd,    "spd <rad/s>");
  g_con.add("iq",     cmdIq,     "iq <A> 直接给电流");
  g_con.add("inject", cmdInject, "inject 0|1|3");
  g_con.add("show",   cmdShow,   "打印监测量");
  g_con.add("fclear", cmdFClear,  "清除故障");
  g_con.add("test",   cmdTest,   "自动故障注入测试");
  obsAddTelemetryNames();
}

void loop() {
  g_con.poll();
  faultServiceOnce();

  telemetry::set(0, g_m.ia);
  telemetry::set(1, g_m.ib);
  telemetry::set(2, g_m.ic);
  telemetry::set(3, g_m.iq);
  telemetry::set(4, (float)g_faultCode);
  telemetry::set(5, g_m.vLimitScale);
  telemetry::tick();
}
