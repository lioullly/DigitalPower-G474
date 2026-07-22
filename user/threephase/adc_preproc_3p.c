#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_3p.h"
#include "dac.h"

// 三相 ADC 预处理 @ 50kHz (ADC ISR 中调用)
// - 电压偏置校正 + RMS
// - 三相电流转 SI
// - 锁相积分器 → g_sin_wt → DAC 调试输出
void adc_preproc_3p(void)
{
    // --- 电压: AC 偏置 2036, Udc 无偏置 ---
    // 必须与 task_protect.c 的 U_line 索引定义一致
    U_line[0] = (float)(adc2_voltage_buffer[ADC2_BUF_UAB] - 2036) * U_coefficient;
    U_line[1] = (float)(adc2_voltage_buffer[ADC2_BUF_UDC] - 0)    * U_coefficient;
    U_line[2] = (float)(adc2_voltage_buffer[ADC2_BUF_UAC] - 2036) * U_coefficient;
    U_line[3] = (float)(adc2_voltage_buffer[ADC2_BUF_UBC] - 2036) * U_coefficient;

    // --- 电流: IL2/IL3 转 SI (IL1 在 ISR 中已转换) ---
    I_line[1] = (float)g_il2 * current_const;
    I_line[2] = (float)g_il3 * current_const;

    // --- 锁相积分器 → g_sin_wt → DAC 调试输出 ---
    float phase = f32_Integral_Calculate(&Sine_Phase_Integrator, 50.0f);
    g_sin_wt = arm_sin_f32(phase * 2.0f * PI);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                     (uint32_t)(g_sin_wt * 2047 + 2048));

    // --- 滑动窗口 RMS: Uab, Ubc, IL1 (1000 点 @ 50kHz = 20ms = 1 电网周期) ---
    static float uab_buf[1000], ubc_buf[1000], il1_buf[1000];
    static uint16_t idx = 0;

    uab_buf[idx] = U_line[0];
    ubc_buf[idx] = U_line[3];
    il1_buf[idx] = I_line[0];

    if (++idx >= 1000) {
        idx = 0;
        arm_rms_f32(uab_buf, 1000, &g_uab_rms);
        arm_rms_f32(ubc_buf, 1000, &g_ubc_rms);
        arm_rms_f32(il1_buf, 1000, &g_irms);
    }
}
