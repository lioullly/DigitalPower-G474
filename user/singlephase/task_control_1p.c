#include "task_control_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"

extern PI_TypeDef Voltage_PI_Loop;
extern PR_TypeDef Current_PR_Loop_alpha;
extern float g_duty_a, g_duty_b, g_duty_c;

// Unified PI(voltage outer) + PR(current inner) control loop
void Task_Control_PI_PR_Loop(void)
{
    // --- edge detection: must be before early return to catch falling edge ---
    static uint8_t prev_run = 0;
    uint8_t rising = (Run_Flag && !prev_run);
    prev_run = Run_Flag;

    if (!Run_Flag) return;
    HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);

    static float i_mag = I_MAG_DEFAULT;
    static uint16_t v_dec = 0;
    static float uab_filt = 0.0f;

    // re-init on Run_Flag rising edge
    if (rising) {
        f32_PI_Init(&Voltage_PI_Loop, 1.0f/10000.0f, 0.25f, 12.0f, (int16_t)I_MAG_MAX, 0);
        f32_PR_Init(&Current_PR_Loop_alpha, 2.0f, 20.0f, 50.0f, 10.0f, 10000.0f, (float)DC_OV, -(float)DC_OV);
        i_mag = I_MAG_DEFAULT;
        v_dec = 0;
        uab_filt = 0.0f;
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_F);
    }

    if (++v_dec >= 200) {  // slow voltage loop, every 20ms
        v_dec = 0;
        float v_err = UREF - g_uab_rms;
        i_mag += 0.02f * v_err;
        if (i_mag > I_MAG_MAX) i_mag = I_MAG_MAX;
        if (i_mag < 0.05f)   i_mag = 0.05f;
    }
    float i_ref = i_mag * 1.414214f * g_sin_wt;  // RMS→peak
    float i_fb  = -I_line[0];
    float i_err = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    uab_filt += 0.1f * (U_line[0] - uab_filt);  // ~160Hz LPF
    float v_ref = v_ctrl + uab_filt;
    float udc   = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    float m = v_ref / udc;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;
    g_dbg_m     = m;

    g_duty_a = 0.5f + 0.5f * m;
    g_duty_b = 0.5f - 0.5f * m;

    if (g_duty_a > 0.95f) g_duty_a = 0.95f;
    if (g_duty_b > 0.95f) g_duty_b = 0.95f;
    if (g_duty_a < 0.05f) g_duty_a = 0.05f;
    if (g_duty_b < 0.05f) g_duty_b = 0.05f;
    g_duty_c = 0.5f;
    Task_PWM_1P_Update();
}
