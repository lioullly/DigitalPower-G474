#include "task_adc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_protect.h"
#include "hrtim.h"
#include "dac.h"

// --- ADC hardware init (call once from user_Init) ---
void Task_ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_voltage_buffer, 4);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);  // DMA in background, no ISR needed

    HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 0);  // 50kHz/1=50kHz
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc4);
    HAL_NVIC_DisableIRQ(ADC4_IRQn);  // ADC4 completes first, read from ADC1 callback if needed

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    f32_Integral_Init(&Sine_Phase_Integrator, 1.0f/48730.0f, 1.0f);
    Sine_Phase_Integrator.x1 = 50.0f;  // pre-charge, avoid half-step on first call
}

// --- Control hook: set by user_Init to point to active control function ---
void (*g_control_isr)(void) = NULL;


