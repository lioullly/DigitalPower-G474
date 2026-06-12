#ifndef __user_h__
#define __user_h__

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "gpio.h"
#include "usart.h"
#include "hardware_def.h"
#include "pi_pr_ctrl.h"

void user_Init(void);
void user_Loop(void);
void Task_Button_Scan(void);

extern uint8_t Run_Flag;
extern PI_TypeDef Voltage_PI_Loop;
extern PR_TypeDef Current_PR_Loop_alpha;
extern float U_line[4], I_line[3];
extern float g_wt;
extern uint8_t g_pll_locked;
extern float g_freq_est;
extern volatile float g_sin_wt;
extern float g_duty_a, g_duty_b, g_duty_c;
extern float g_uab_rms, I_mag, g_irms, U_coefficient, current_const;
extern float Iref_alpha, Iref_beta;
extern volatile int32_t g_il1, g_il2, g_il3;
extern volatile uint8_t g_adc_data_ready;
extern uint16_t adc2_voltage_buffer[4];
extern Integral_TypeDef Sine_Phase_Integrator;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern float g_dbg_err, g_dbg_vctrl;
extern float g_pfc_phase_deg;
extern float g_grid_phi_deg;

#endif
