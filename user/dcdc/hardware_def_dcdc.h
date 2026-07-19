#ifndef __HARDWARE_DEF_DCDC_H
#define __HARDWARE_DEF_DCDC_H

#include "../hardware_def.h"

// --- Buck DCDC 硬件参数 ---
#define BUCK_L              220e-6f             // 电感 [H]
#define BUCK_COUT           (2*470e-6f + 2*47e-6f) // 输出电容 [F]: 2×470uF电解 + 2×47uF MLCC
#define BUCK_CIN            BUCK_COUT           // 输入电容 [F]: 同输出

#define BUCK_UREF           12.0f   // output voltage reference [V]
#define BUCK_VIN_MIN        15.0f   // min input voltage to start [V]
#define BUCK_IL_MAX          2.0f   // max inductor current [A]
#define BUCK_DUTY_MAX        0.90f  // max duty cycle
#define BUCK_DUTY_ADJ_MAX    0.20f  // max PI duty adjustment from feedforward

// Buck PI 参数 (电压外环1kHz→A, 电流内环10kHz→占空比, 需在实物上整定)
#define BUCK_V_KP            0.5f   // 电压环 Kp (1V误差→0.5A)
#define BUCK_V_KI            5.0f   // 电压环 Ki
#define BUCK_I_KP           0.005f  // 电流环 Kp (1A误差→0.5%占空比)
#define BUCK_I_KI           0.1f    // 电流环 Ki

// Buck 固定占空比调试 (Task_Debug_Buck)
#define BUCK_DEBUG_DUTY       0.50f  // 固定占空比 [0~1]

// Buck 开环电流参考 (电压外环未启用时使用)
#define BUCK_IREF_OPEN        1.0f   // 固定电流参考 [A]
#define BUCK_IREF_SLEW        0.005f // 电流参考斜坡步长 [A] → 50A/s

// Buck soft-start (per call @ 10kHz, 0.1ms)
#define BUCK_VREF_SLEW       0.006f // Vref 斜坡步长 [V] → 60V/s, 12V约200ms (电压环用)
#define BUCK_IL_START        0.5f   // 初始电流上限 [A]
#define BUCK_IL_SLEW         0.002f // 电流上限斜坡步长 [A] → 20A/s

// Buck ADC channel mapping (adc2_voltage_buffer[] index, DC采样无偏置)
#define BUCK_ADC_VIN_BUF     0      // ADC2 IN1 → buffer[0] = 输入电压
#define BUCK_ADC_VOUT_BUF    1      // ADC2 IN2 → buffer[1] = 输出电压
#define BUCK_ADC_IL_IDX      0      // I_line[0] = 电感电流 (ADC1 injected)

#endif
