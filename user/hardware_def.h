#ifndef __hardware_def_h
#define __hardware_def_h

#include "arm_math.h"

// --- ADC scaling (公共) ---
#define VOLTAGE_GAIN       (4.7f/200.0f)
#define VOLTAGE_CONST      (3.3f/(VOLTAGE_GAIN*4095))

#define R                  (0.020f)
#define CURRENT_GAIN       (8.2f)
#define CURRENT_CONST      (3.3f/(R*CURRENT_GAIN*2048))  // diff: ±2048 counts

// --- DC bus voltage protection (公共，单相+DCDC 共用阈值) ---
#define DC_OV              80.0f   // DC bus overvoltage threshold (V)
#define DC_UV               0.0f   // DC bus undervoltage threshold (V)

// --- current protection (公共，所有拓扑共用) ---
#define IL1_OC             9.0f  // overcurrent threshold (temporarily raised for debug)

// --- PR current loop output clamp (公共) ---
#define PR_CTRL_CLAMP        20.0f  // v_ctrl output limit [V]

#endif
