#ifndef __hardware_def_h
#define __hardware_def_h

#include "arm_math.h"

// --- ADC voltage scaling ---
// Vout = Vin × Rbottom/(Rtop+Rbottom),  V_per_count = Vref / (GAIN × 4095)
// 200k:4.7k 分压, AC 有 1.65V 偏置, DC 无偏置
#define V_RTOP             (200.0f)
#define V_RBOTTOM          (4.7f)
#define VOLTAGE_GAIN       (V_RBOTTOM/(V_RTOP+V_RBOTTOM))
#define VOLTAGE_CONST      (3.25f/(VOLTAGE_GAIN*4095))

#define R                  (0.020f)
#define CURRENT_GAIN       (8.2f)
#define CURRENT_CONST      (3.3f/(R*CURRENT_GAIN*2048))  // diff: ±2048 counts

// --- DC bus voltage protection (公共，单相+DCDC 共用阈值) ---
#define DC_OV              75.0f   // DC bus overvoltage threshold (V)
#define DC_UV               0.0f   // DC bus undervoltage threshold (V)

// --- current protection (公共，所有拓扑共用) ---
#define IL1_OC             10.5f  // overcurrent threshold [A]

// --- PR current loop output clamp (公共) ---
#define PR_CTRL_CLAMP        20.0f  // v_ctrl output limit [V]

#endif
