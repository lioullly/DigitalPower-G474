#ifndef __hardware_def_h
#define __hardware_def_h

#include "arm_math.h"

// --- ADC scaling ---
#define VOLTAGE_GAIN       (4.7f/200.0f)
#define VOLTAGE_CONST      (3.3f/(VOLTAGE_GAIN*4095))
#define UAB_VOLTAGE_CONST  (VOLTAGE_CONST * 1.0f)   // same divider as Udc

#define R                  (0.020f)
#define CURRENT_GAIN       (8.2f)
#define CURRENT_CONST      (3.3f/(R*CURRENT_GAIN*2048))  // diff: ±2048 counts

// --- inverter LC filter ---
#define L                  (0.001000f)
#define Fsw                (50.0f)
#define K_CONST            (2.0f*PI*Fsw*L)

// --- DC bus protection ---
#define DC_OV              60.0f   // DC bus overvoltage threshold (V)
#define DC_UV               0.0f   // DC bus undervoltage threshold (V)

// --- current protection ---
#define IL1_OC              5.0f   // overcurrent RMS threshold (A)

// --- off-grid voltage control (task_control_1p.c) ---
#define OFFGRID_UREF        20.0f   // AC output RMS voltage reference [V]
#define I_MAG_DEFAULT        1.0f   // initial Vpeak [V]
#define I_MAG_MAX           10.0f   // PI output upper clamp [Vpeak]
#define OFFGRID_MOD_INDEX    0.5f

// --- PFC current control (task_control_pfc.c) ---
#define PFC_UREF            40.0f   // DC bus voltage reference [V] (for future voltage loop)
#define PFC_VPEAK      (20.0f * 1.414f)  // nominal AC peak voltage [V]
#define PFC_K          (1.0f / PFC_VPEAK) // Uab→normalized gain

#endif
