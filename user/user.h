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
void Task_Display(void);

extern uint8_t Run_Flag;
extern PI_TypeDef Voltage_PI_Loop;
extern PR_TypeDef Current_PR_Loop_alpha;
extern float U_line[2], I_line[1];
extern float g_wt;
extern uint8_t g_pll_locked;
extern float g_freq_est;
extern volatile float g_sin_wt;
extern float g_duty_a, g_duty_b, g_duty_c;
extern float g_uab_rms, g_ubc_rms, I_mag, g_irms, U_coefficient, current_const;
extern float Iref_alpha, Iref_beta;
extern volatile int32_t g_il1;
extern volatile uint8_t g_adc_data_ready;
extern volatile uint16_t adc2_voltage_buffer[2];
extern Integral_TypeDef Sine_Phase_Integrator;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern float g_dbg_err, g_dbg_vctrl, g_isr_khz;
extern float g_dbg_iref, g_dbg_iref_inst, g_dbg_m;
extern const char *g_task_name;
extern float g_pfc_phase_deg;
extern float g_grid_phi_deg;
extern uint8_t g_oled_ok;
extern void (*g_adc_preproc)(void); // ADC ISR 预处理 @50kHz (电压填充/锁相/DAC/RMS)
extern void (*g_display_fn)(void);  // OLED 显示函数
extern void (*g_vofa_fn)(void);     // VOFA 采集函数

// 各拓扑的 ADC 预处理函数
void adc_preproc_1p(void);

#endif
