#ifndef __hardware_def_h
#define __hardware_def_h

#include "arm_math.h"

// --- ADC scaling ---
#define VOLTAGE_GAIN       (4.7f/200.0f)
#define VOLTAGE_CONST      (3.3f/(VOLTAGE_GAIN*4095))

#define R                  (0.020f)
#define CURRENT_GAIN       (8.2f)
#define CURRENT_CONST      (3.3f/(R*CURRENT_GAIN*2048))  // diff: ±2048 counts

// --- inverter LC filter ---
#define L                  (0.001000f)
#define Fsw                (50.0f)
#define K_CONST            (2.0f*PI*Fsw*L)

// --- DC bus protection ---
#define DC_OV              80.0f   // DC bus overvoltage threshold (V)
#define DC_UV               0.0f   // DC bus undervoltage threshold (V)

// --- current protection ---
#define IL1_OC              9.0f  // overcurrent instantaneous I threshold (A)

// --- PR current loop output clamp (all modes) ---
#define PR_CTRL_CLAMP        20.0f  // v_ctrl output limit [V]

// --- off-grid inverter (task_control_1p.c) ---
#define OFFGRID_UREF        32.0f   // AC output RMS voltage reference [V]
#define OFFGRID_IREF_MAX     5.0f   // max RMS current [A], peak ~7A
#define OFFGRID_UDC_MIN     45.0f   // min DC bus to start [V]
#define OFFGRID_PR_CLAMP    20.0f   // PR v_ctrl clamp [V]
#define I_MAG_DEFAULT        1.0f   // legacy: initial rms current [A]
#define I_MAG_MAX           10.0f   // legacy: PI upper clamp

// --- grid-tied inverter (task_control_grid.c) ---
#define GRID_UREF_MIN   10.0f   // min grid RMS to attempt sync [V]
#define GRID_I_MAG_DEFAULT   1.0f   // initial active rms current [A]
#define GRID_IREF_MAX         5.0f  // max RMS current [A], peak = ×1.414
#define GRID_PHI_DEG_DEFAULT 0.0f   // initial power angle [°], 0=UPF

// --- PFC current control (task_control_pfc.c) ---
#define PFC_UREF            40.0f   // DC bus voltage reference [V]
#define PFC_IREF_MAX         5.66f  // max RMS current [A], peak ~8A
#define PFC_PHASE_DEG_DEFAULT 0.0f  // initial phase shift [°]





#endif
