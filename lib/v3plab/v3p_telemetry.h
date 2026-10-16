/**
 * @file v3p_telemetry.h
 * @brief 二进制波形遥测（E0.2 上位机链路）
 *
 * 帧格式：
 *   0xAA 0x55 | nch(u8) | id(u8) | float32 × nch | crc16(u16, 小端)
 *   CRC16-CCITT (poly 0x1021, init 0xFFFF)，覆盖 nch..payload
 *
 * 为什么用二进制：Serial.print 文本在 100 kHz 采样下会把控制环拖垮。
 * 用法：
 *   控制环里：telemetry::set(0, theta); telemetry::set(1, omega); ...
 *   低优先级任务里：telemetry::tick();   // 非阻塞，缓冲满了就丢帧
 */
#pragma once

#include <stdint.h>

namespace v3p {
namespace telemetry {

constexpr int MAX_CH = 16;

/// 初始化（不会启动任务，任务由实验自己建，便于控制放在哪个核）
void begin(uint32_t baud = 2000000);

void setChannels(uint8_t n);
void setChannelName(int idx, const char* name);
void setFrameId(uint8_t id);

/// 控制环里调用（只写内存，极快）
inline void set(int idx, float v);

/// 便捷：一次性写多路
void setAll(const float* v, int n);

/// 低优先级任务里调用：打包 + 非阻塞发送
void tick();

/// 统计
uint32_t framesSent();
uint32_t framesDropped();

/// 供实现使用（inline 版本需要可见的数组）
extern volatile float g_ch[MAX_CH];
extern volatile uint8_t g_nch;

inline void set(int idx, float v) {
  if (idx >= 0 && idx < MAX_CH) g_ch[idx] = v;
}

}  // namespace telemetry
}  // namespace v3p
