#include "v3p_telemetry.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

namespace v3p {
namespace telemetry {

volatile float   g_ch[MAX_CH] = {0};
volatile uint8_t g_nch = 0;

static uint8_t  s_id = 0;
static char     s_names[MAX_CH][12];
static uint32_t s_sent = 0;
static uint32_t s_drop = 0;

static uint16_t crc16(const uint8_t* p, uint32_t n) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void begin(uint32_t baud) {
  g_nch = 0;
  s_id  = 0;
  s_sent = s_drop = 0;
#if defined(ARDUINO_ARCH_ESP32)
  Serial.begin(baud);
#endif
}

void setChannels(uint8_t n) {
  if (n > MAX_CH) n = MAX_CH;
  g_nch = n;
}

void setChannelName(int idx, const char* name) {
  if (idx < 0 || idx >= MAX_CH || !name) return;
  int i = 0;
  for (; name[i] && i < 11; i++) s_names[idx][i] = name[i];
  s_names[idx][i] = '\0';
}

void setFrameId(uint8_t id) { s_id = id; }

void setAll(const float* v, int n) {
  if (!v) return;
  if (n > MAX_CH) n = MAX_CH;
  for (int i = 0; i < n; i++) g_ch[i] = v[i];
}

void tick() {
  const uint8_t n = g_nch;
  if (n == 0) return;

#if defined(ARDUINO_ARCH_ESP32)
  // 非阻塞：发送缓冲剩余不足一帧就直接丢（宁可丢帧也不要阻塞控制环）
  const uint32_t need = 4u + 4u * n + 2u;
  if ((uint32_t)Serial.availableForWrite() < need) { s_drop++; return; }

  uint8_t hdr[4] = {0xAA, 0x55, n, s_id};
  Serial.write(hdr, 4);

  uint8_t payload[MAX_CH * 4];
  for (uint8_t i = 0; i < n; i++) {
    const float f = g_ch[i];
    uint32_t u;
    __builtin_memcpy(&u, &f, 4);
    payload[i * 4 + 0] = (uint8_t)(u & 0xFF);
    payload[i * 4 + 1] = (uint8_t)((u >> 8) & 0xFF);
    payload[i * 4 + 2] = (uint8_t)((u >> 16) & 0xFF);
    payload[i * 4 + 3] = (uint8_t)((u >> 24) & 0xFF);
  }
  Serial.write(payload, (size_t)n * 4);

  // CRC 覆盖 nch..payload
  uint8_t crcbuf[4 + MAX_CH * 4];
  crcbuf[0] = n;
  crcbuf[1] = s_id;
  __builtin_memcpy(&crcbuf[2], payload, (size_t)n * 4);
  const uint16_t crc = crc16(crcbuf, 2u + 4u * n);
  uint8_t tail[2] = {(uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8)};
  Serial.write(tail, 2);

  s_sent++;
#else
  s_drop++;
#endif
}

uint32_t framesSent()    { return s_sent; }
uint32_t framesDropped() { return s_drop; }

}  // namespace telemetry
}  // namespace v3p
