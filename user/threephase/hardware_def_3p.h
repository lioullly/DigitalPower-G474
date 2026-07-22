#ifndef __HARDWARE_DEF_3P_H
#define __HARDWARE_DEF_3P_H

#include "../hardware_def.h"

// ============================================================
// 三相并网/离网逆变器参数
// ============================================================

// --- ADC 通道映射 (adc2_voltage_buffer[] 索引) ---
// 必须与单相 U_line 索引保持一致（共用 task_protect.c 的 mask）
// U_line[0]=Uab, U_line[1]=Udc, U_line[2]=Uac, U_line[3]=Ubc
#define ADC2_BUF_UAB  0    // ADC2 IN1 → Uab (AC, 偏置 2036)
#define ADC2_BUF_UDC  1    // ADC2 IN2 → Udc (DC, 无偏置)
#define ADC2_BUF_UAC  2    // ADC2 IN3 → Uac (AC, 偏置 2036)
#define ADC2_BUF_UBC  3    // ADC2 IN4 → Ubc (AC, 偏置 2036)

// --- 电压环 (双 PI: Uab + Ubc 线电压 RMS 控制) ---
#define GRID3P_UREF        32.0f   // 线电压 RMS 参考 [V]
#define GRID3P_V_KP         0.01f  // 电压环 Kp
#define GRID3P_V_KI         0.05f  // 电压环 Ki
#define GRID3P_V_DECIMATE   10     // 10kHz→1kHz 降采样比
#define GRID3P_IMAG_MAX      5.0f  // 电压 PI 输出上限 [A RMS]

// --- 电流环 (3× PR, 每相独立) ---
// 三相共用一个 PR 参数集，各自实例独立运行
#define GRID3P_PR_KP        4.0f   // PR 比例增益
#define GRID3P_PR_KR       10.0f   // PR 谐振增益
#define GRID3P_PR_F0       50.0f   // 谐振频率 [Hz]
#define GRID3P_PR_BW       10.0f   // 带宽 [Hz]
#define GRID3P_IREF_PK     10.0f   // 瞬时电流峰值限幅 [A] (≈7A RMS)

// --- PLL (SRF-PLL, Clarke→Park 锁相) ---
#define PLL_KP            60.0f    // 锁相环比例增益
#define PLL_KI             2.0f    // 锁相环积分增益
#define PLL_LOCK_CNT      200      // 锁定计数 (200ms @ 1kHz)
#define PLL_VMAG_MIN       10.0f   // 最小电网电压幅值 [V]

// --- 激活条件 (带迟滞) ---
#define GRID3P_VMAG_START  20.0f   // 启动: v_mag > 20V
#define GRID3P_VMAG_HOLD   15.0f   // 保持: v_mag > 15V
#define GRID3P_UDC_START    45.0f  // 启动: Udc > 45V
#define GRID3P_UDC_HOLD     35.0f  // 保持: Udc > 35V

// --- 软启动 ---
#define GRID3P_I_START      0.1f   // 初始电流幅值 [A]
#define GRID3P_I_SLEW       0.002f // 电流 ramp 步长 [A] → 20A/s @ 10kHz

// --- 调制 ---
#define GRID3P_M_MAX        0.95f  // 最大调制比
#define GRID3P_M_MIN        0.05f  // 最小调制比

#endif
