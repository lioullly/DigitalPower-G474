#ifndef __user_h__
#define __user_h__

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include "hardware_def.h"
#include "pi_pr_ctrl.h"

void user_Init(void);
void user_Loop(void);
void Task_Control_Debug(void);
void Task_Control_OffGrid(void);
void Task_Control_GridTied(void);
void Task_Button_Scan(void);

extern uint8_t Run_Flag;

extern PI_TypeDef   Voltage_PI_Loop;
extern PR_TypeDef   Current_PR_Loop_alpha;
extern PR_TypeDef   Current_PR_Loop_beta;

extern volatile int32_t g_il1;
extern volatile int32_t g_il2;
extern volatile int32_t g_il3;
extern volatile uint8_t  g_adc_data_ready;

extern uint16_t adc2_voltage_buffer[4];

extern float U_coefficient;
extern float current_const;
extern float K_coefficient;

extern float Uref;
extern float U_line[4];
extern float I_line[3];

extern float Iref_alpha, Iref_beta;
extern float g_duty_a, g_duty_b, g_duty_c;

extern float g_wt;

#endif
