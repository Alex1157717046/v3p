/**
 * @file v3p_console.h
 * @brief 串口命令行（用于实验过程中在线改参数，不必反复烧录）
 *
 * 命令表由实验通过 add() 注册，poll() 在 loop() 里调用。
 * 命令语法：<name> [arg1] [arg2] ...
 */
#pragma once

#include <stdint.h>

namespace v3p {

class Console {
 public:
  using Handler = void (*)(int argc, char** argv);

  static constexpr int MAX_CMDS = 24;

  void begin(uint32_t baud = 115200);
  bool add(const char* name, Handler fn, const char* help = nullptr);

  /// 在 loop() 里调用；收到完整行就执行
  void poll();

  /// 打印
  static void print(const char* s);
  static void printfln(const char* fmt, ...);

  /// 取第 i 个参数为数值（失败返回默认值）
  static float  argf(char** argv, int argc, int i, float def);
  static int32_t argi(char** argv, int argc, int i, int32_t def);

 private:
  struct Entry { const char* name; Handler fn; const char* help; };
  Entry   cmds_[MAX_CMDS];
  int     n_ = 0;
  char    buf_[96];
  int     len_ = 0;
  bool    dirty_ = false;   ///< 本行混入了二进制字节（遥测帧），整行丢弃
};

}  // namespace v3p
