#include "v3p_console.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#endif

namespace v3p {

void Console::begin(uint32_t baud) {
#if defined(ARDUINO_ARCH_ESP32)
  Serial.begin(baud);
#endif
  n_ = 0;
  len_ = 0;
}

bool Console::add(const char* name, Handler fn, const char* help) {
  if (n_ >= MAX_CMDS || !name || !fn) return false;
  cmds_[n_].name = name;
  cmds_[n_].fn   = fn;
  cmds_[n_].help = help;
  n_++;
  return true;
}

/**
 * 逐字节读串口。
 * 关键：遥测（二进制帧）和命令行共用同一个 USB 串口，
 * 所以这里把"含不可打印字符的行"整行丢弃，避免二进制数据被误当成命令执行。
 */
void Console::poll() {
#if defined(ARDUINO_ARCH_ESP32)
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;

    if (c == '\n' || c == '\r') {
      if (len_ > 0 && !dirty_) {
        buf_[len_] = '\0';
        // 切词
        char* argv[8];
        int argc = 0;
        char* p = buf_;
        while (*p && argc < 8) {
          while (*p == ' ' || *p == '\t') p++;
          if (!*p) break;
          argv[argc++] = p;
          while (*p && *p != ' ' && *p != '\t') p++;
          if (*p) *p++ = '\0';
        }
        if (argc > 0) {
          bool found = false;
          for (int i = 0; i < n_; i++) {
            if (strcmp(cmds_[i].name, argv[0]) == 0) {
              cmds_[i].fn(argc, argv);
              found = true;
              break;
            }
          }
          if (!found && strcmp(argv[0], "help") == 0) {
            Console::print("commands:\r\n");
            for (int i = 0; i < n_; i++) {
              Console::print("  ");
              Console::print(cmds_[i].name);
              if (cmds_[i].help) { Console::print(" - "); Console::print(cmds_[i].help); }
              Console::print("\r\n");
            }
          } else if (!found) {
            Console::print("unknown cmd\r\n");
          }
        }
      }
      len_ = 0;
      dirty_ = false;
    } else {
      if (c >= 32 && c < 127) {
        if (len_ < (int)sizeof(buf_) - 1) buf_[len_++] = (char)c;
      } else {
        dirty_ = true;   // 二进制帧混进来了，整行作废
      }
    }
  }
#endif
}

void Console::print(const char* s) {
#if defined(ARDUINO_ARCH_ESP32)
  Serial.print(s);
#else
  (void)s;
#endif
}

void Console::printfln(const char* fmt, ...) {
#if defined(ARDUINO_ARCH_ESP32)
  char tmp[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  Serial.print(tmp);
  Serial.print("\r\n");
#else
  (void)fmt;
#endif
}

float Console::argf(char** argv, int argc, int i, float def) {
  if (i >= argc) return def;
  return strtof(argv[i], nullptr);
}

int32_t Console::argi(char** argv, int argc, int i, int32_t def) {
  if (i >= argc) return def;
  return (int32_t)strtol(argv[i], nullptr, 0);
}

}  // namespace v3p
