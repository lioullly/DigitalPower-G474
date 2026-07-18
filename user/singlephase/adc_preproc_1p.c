#include "user.h"
#include "pi_pr_ctrl.h"
#include "dac.h"

// 单相 ADC 预处理 @ 50kHz (ADC ISR 中调用)
// 电压填充(AC偏置) + 锁相积分 + DAC 调试输出 + 滑动 RMS
void adc_preproc_1p(void)
{
    // 电压: Uab 有 AC 偏置, Udc 无偏置
    U_line[0] = (float)(adc2_voltage_buffer[0] - 2036) * U_coefficient;
    U_line[1] = (float)(adc2_voltage_buffer[1] - 0)    * U_coefficient;

    // 锁相积分器 → g_sin_wt → DAC 调试输出
    float phase = f32_Integral_Calculate(&Sine_Phase_Integrator, 50.0f);
    g_sin_wt = arm_sin_f32(phase * 2.0f * PI);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                     (uint32_t)(g_sin_wt * 2047 + 2048));

    // 滑动窗口 RMS — 1000 点 @ 50kHz = 1 个电网周期 (20ms)
    static float uab_buf[1000], il1_buf[1000];
    static uint16_t idx = 0;
    uab_buf[idx] = U_line[0];
    il1_buf[idx] = I_line[0];
    if (++idx >= 1000) {
        idx = 0;
        arm_rms_f32(uab_buf, 1000, &g_uab_rms);
        arm_rms_f32(il1_buf, 1000, &g_irms);
    }
}
