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

#define UDC_OV   60.0f
#define UDC_UV    0.0f
#define IL1_OC    5.0f

#define UREF            40.0f  // PFC bus voltage reference (V)
#define I_MAG_DEFAULT   1.0f   // default current magnitude (A)
#define I_MAG_MAX       5.0f   // PI output upper clamp (A), must be < IL1_OC

#define UAC_OV_RATIO  1.6f  // AC overvoltage: |Uab| > UREF * 1.5
#define OFFGRID_MOD_INDEX  0.5f

#endif
