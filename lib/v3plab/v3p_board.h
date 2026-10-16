/**
 * @file v3p_board.h
 * @brief DengFOC V3P 驱动板 —— 板级常量（依据 4、原理图与PCB/V3P驱动板最新版原理图.pdf 核对）
 *
 * 核对结论（2026-10）：
 *   主控        : ESP32（背插 WEMOS LOLIN32 Lite）或 STM32 转接板 + STM32F103C8T6
 *   功率级      : 每路 6× WSD4070DN 分立 NMOS + 1× FD6287T 三相半桥栅极驱动
 *   栅极电阻    : 2.2 Ω（R26 等），栅源 10 kΩ 下拉
 *   自举        : 外置 1 µF（C44/C45/C47）+ 1N5819WS（D3/D4/D5），VB 供电 10 V
 *   ★ 关键       : FD6287T 的 HIN1~3 与 LIN1#~3# 在板上被接成同一组网络
 *                  → MCU 只有 3 路 PWM/相，死区由驱动 IC 内部固定，MCU 无法设置
 *   电流采样    : INA240A2PWR ×4（增益 50 V/V）+ 0.01 Ω 分流（inline 相线采样）
 *   位置反馈    : AS5600（I2C）×2，引脚复用支持 ABI / SPI / HALL
 *   电源        : TPS54302DDCR(5V) + BL9342(10V, 栅极驱动) + XC6210B332MR(3.3V)
 */
#pragma once

#include <stdint.h>

namespace v3p {

// ============================ 功率级：三相 PWM ============================
// 注意：每路只有 3 根 PWM 线 → FD6287T 的 HIN_i 与 LIN#_i 并联
constexpr int M0_IN1 = 32;
constexpr int M0_IN2 = 33;
constexpr int M0_IN3 = 25;
constexpr int M1_IN1 = 26;
constexpr int M1_IN2 = 27;
constexpr int M1_IN3 = 14;

/// 驱动板使能：拉高才有输出（新版 V3P 必须）
constexpr int MOTOR_EN = 12;

// ============================ 电流采样 ============================
constexpr int M0_OUT1_CS = 39;  // ADC1_CH3 (VN)
constexpr int M0_OUT2_CS = 36;  // ADC1_CH0 (VP)
constexpr int M1_OUT1_CS = 35;  // ADC1_CH7
constexpr int M1_OUT2_CS = 34;  // ADC1_CH6

constexpr float R_SHUNT   = 0.010f;         // 0.01 Ω（丝印 R010），可换 0.001 Ω
constexpr float INA_GAIN  = 50.0f;          // INA240A2 = 50 V/V
constexpr float CSA_V_PER_A = R_SHUNT * INA_GAIN;  // 0.5 V/A
constexpr float ADC_VREF  = 3.3f;           // 理论满量程（实测需标定）
constexpr float I_FULL_SCALE = 3.3f;        // ±3.3 A @ 0.01 Ω

// ============================ 编码器 / 复用引脚 ============================
constexpr int SDA_0 = 19;   // M0 I2C 数据（ABI-A / SPI-MISO / HALL-A）
constexpr int SCL_0 = 18;   // M0 I2C 时钟（ABI-B / SPI-SCLK / HALL-B）
constexpr int SDA_1 = 23;   // M1 I2C 数据（SPI-MOSI）
constexpr int SCL_1 = 5;    // M1 I2C 时钟（SPI-SS）
constexpr int IO_15 = 15;   // 备用（HALL-C of M0）
constexpr int IO_13 = 13;   // 备用（HALL-C of M1）

// ============================ 其他 ============================
constexpr int POWER_ADC = 4;    // 母线电压检测（ADC2_CH0，与 WiFi 冲突！）
constexpr int UART2_TX  = 17;   // 板载排针 TXD（备用串口，非 USB 串口）
constexpr int UART2_RX  = 16;   // 板载排针 RXD

// ============================ 默认电机参数（2208 云台电机，需实测替换）============================
constexpr int   POLE_PAIRS_DEFAULT = 7;
constexpr float VDC_DEFAULT        = 12.0f;   // V
constexpr float RS_DEFAULT         = 10.0f;   // Ω（量级估算，E5.1 实测替换）
constexpr float LS_DEFAULT         = 0.010f;  // H（量级估算，E5.2 实测替换）
constexpr float LAMBDA_DEFAULT     = 0.0079f; // Wb（量级估算，E4.1 实测替换）

// ============================ 编译期开关 ============================
/// 取消注释可把"母线电压"当成实测值来源（默认与官方例程一致：直接用常数）
// #define V3P_USE_POWER_ADC 1

}  // namespace v3p
