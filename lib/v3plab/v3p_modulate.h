/**
 * @file v3p_modulate.h
 * @brief 调制方式统一入口：开环实验（E1.x）都通过它选调制策略
 */
#pragma once

#include "v3p_math.h"

namespace v3p {

enum class Mod {
  SPWM,        ///< E1.2
  SVPWM7,      ///< E1.3 七段式
  SVPWM5,      ///< E1.4 五段式 / DPWM
  SVPWMMinMax, ///< E1.3 交叉验证用的载波比较式
  SIXSTEP,     ///< E1.1 六步换相
};

/**
 * @param v         αβ 电压参考（V，幅值 = 相电压峰值）
 * @param vdc       母线电压（V）
 * @param step6     六步模式下的换相步（0~5）
 * @param sector    输出：SVPWM 扇区号（1~6）
 * @param m         输出：调制度 m = |v| / (Vdc/√3)
 */
inline Duty modulate(Mod mode, const AlphaBeta& v, float vdc,
                     int step6 = 0, int* sector = nullptr, float* m = nullptr) {
  switch (mode) {
    case Mod::SIXSTEP: {
      float a, b, c;
      sixStep(step6, &a, &b, &c);
      if (sector) *sector = step6;
      if (m) *m = 0.0f;
      return Duty{a, b, c};
    }
    case Mod::SVPWM5:
      return svpwm5(v, vdc);
    case Mod::SVPWMMinMax:
      return svpwmMinMax(v, vdc);
    case Mod::SPWM:
      return spwm(v, vdc);
    case Mod::SVPWM7:
    default:
      return svpwm7(v, vdc, sector, m);
  }
}

inline const char* modName(Mod m) {
  switch (m) {
    case Mod::SPWM:        return "SPWM";
    case Mod::SVPWM7:      return "SVPWM7";
    case Mod::SVPWM5:      return "SVPWM5";
    case Mod::SVPWMMinMax: return "SVPWM-MM";
    case Mod::SIXSTEP:     return "SIXSTEP";
    default:               return "?";
  }
}

}  // namespace v3p
