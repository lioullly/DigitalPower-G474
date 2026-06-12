#include "task_adc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
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

    HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 4);  // 50kHz/5=10kHz
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // ADC4 disabled - kills scheduler on bare board

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    f32_Integral_Init(&Sine_Phase_Integrator, 1.0f/10000.0f, 1.0f);
    Sine_Phase_Integrator.x1 = 50.0f;  // pre-charge, avoid half-step on first call
}

// --- ADC data fetch → SI units + RMS (scheduled @ 50kHz) ---
void Task_ADC_Fetch(void)
{
    if (!g_adc_data_ready) return;
    g_adc_data_ready = 0;

    I_line[0] = (float)g_il1 * current_const;
    U_line[0] = (float)(adc2_voltage_buffer[0] - 2036) * U_coefficient;        // PA0 = Uab
    U_line[1] = (float)(adc2_voltage_buffer[1] - 0) * U_coefficient;          // PA1 = Udc

    static float uab_buf[200], il1_buf[200];
    static uint16_t idx = 0;
    uab_buf[idx] = U_line[0];
    il1_buf[idx] = I_line[0];
    if (++idx >= 200) {
        idx = 0;
        arm_rms_f32(uab_buf, 200, &g_uab_rms);
        arm_rms_f32(il1_buf, 200, &g_irms);
    }
}

// --- ADC1 Injected ISR: read current, update phase, output DAC ---
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 2050;

    float phase = f32_Integral_Calculate(&Sine_Phase_Integrator, 50.0f);
    g_sin_wt = arm_sin_f32(phase * 2.0f * PI);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(g_sin_wt * 2047 + 2048));

    g_adc_data_ready = 1;
}
