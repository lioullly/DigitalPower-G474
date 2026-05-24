#ifndef __user_h__
#define __user_h__

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "gpio.h"
#include "tim.h"
#include "hardware_def.h"
#include "pi_pr_ctrl.h"

void user_Init(void);
void user_Loop(void);
void Task_Control_Debug(void);
void Task_Button_Scan(void);

extern uint8_t Run_Flag;
extern float U_line[4], I_line[3];
extern float g_wt, g_duty_a, g_duty_b, g_duty_c;
extern float g_uab_rms, I_mag, U_coefficient, current_const, K_coefficient;
extern float Iref_alpha, Iref_beta;
extern volatile int32_t g_il1, g_il2, g_il3;
extern volatile uint8_t g_adc_data_ready;
extern uint16_t adc2_voltage_buffer[4];

#endif
