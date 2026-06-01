#ifndef __hardware_def_h
#define __hardware_def_h

#include "arm_math.h"

#define VOLTAGE_GAIN      (4.7f/200.0f)
#define VOLTAGE_CONST     (3.3f/(VOLTAGE_GAIN*4095))
#define UAB_VOLTAGE_CONST  (VOLTAGE_CONST * 1.0f)   // same divider as Udc

#define R				(0.020f)
#define CURRENT_GAIN 	(8.2f)
#define	CURRENT_CONST 	(3.3f/(R*CURRENT_GAIN*2048))  // diff: ±2048 counts

#define L				(0.001000f)
#define	Fsw				(50.0f)
#define K_CONST			(2.0f*PI*Fsw*L) 

// --- DC bus protection ---
#define DC_OV    80.0f   // DC bus overvoltage threshold (V)
#define DC_UV     0.0f   // DC bus undervoltage threshold (V)

// --- current protection ---
#define IL1_OC_RMS  5.0f   // overcurrent RMS threshold (A)

#define UREF            40.0f  // PFC bus voltage reference (V)
#define I_MAG_DEFAULT   1.0f   // default current magnitude (A)
#define I_MAG_MAX       10.0f   // PI output upper clamp (A), must be < IL1_OC
#define OFFGRID_MOD_INDEX  0.5f

#endif
