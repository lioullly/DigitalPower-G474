#ifndef __HARDWARE_DEF_1P_H
#define __HARDWARE_DEF_1P_H

#include "../hardware_def.h"

// --- off-grid inverter (task_control_1p.c) ---
#define OFFGRID_UREF        10.0f   // AC output RMS voltage reference [V]
#define OFFGRID_IREF_MAX     5.0f   // max RMS current [A], peak ~7A
#define OFFGRID_UDC_MIN     30.0f   // min DC bus to start [V]
#define I_MAG_DEFAULT        1.0f   // initial rms current [A]
#define I_MAG_MAX           10.0f   // PI upper clamp [A]

// --- grid-tied inverter (task_control_grid.c) ---
#define GRID_UREF_MIN       10.0f   // min grid RMS to attempt sync [V]
#define GRID_I_MAG_DEFAULT   1.0f   // initial active rms current [A]
#define GRID_IREF_MAX         5.0f  // max RMS current [A], peak = ×1.414
#define GRID_PHI_DEG_DEFAULT 0.0f  // initial power angle [°], 0=UPF

// --- PFC (task_control_pfc.c) ---
#define PFC_UREF            65.0f   // DC bus voltage reference [V]
#define PFC_IREF_MAX         7.0f  // max RMS current [A], peak ~9.9A
#define PFC_PHASE_DEG_DEFAULT 0.0f  // initial phase shift [°]

#endif
