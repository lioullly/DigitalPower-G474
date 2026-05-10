#ifndef __hardware_def_h
#define __hardware_def_h

#include "arm_math.h"

#define VOLTAGE_GAIN    (4.7f/200.0f)	//???γλ??
#define VOLTAGE_CONST   (3.3f/(VOLTAGE_GAIN*4095))

#define R				(0.020f)								//????????
#define CURRENT_GAIN 	(8.2f)					//???γλ??
#define	CURRENT_CONST 	(3.3f/(R*CURRENT_GAIN*8190)) //2bit right-shitf & 64xRatio

#define L				(0.001000f)						//???
#define	Fsw				(50.0f)						//???????
// K_CONST = wL, DQ ??????, ??? ????+PR ???¦Δ???, ????????
#define K_CONST			(2.0f*PI*Fsw*L)  //2*PI*Fsw*L

#define UDC_OV   80.0f
#define UDC_UV    10.0f
#define IL1_OC    5.0f

#endif
