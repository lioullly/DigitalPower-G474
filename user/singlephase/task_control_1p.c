#include "task_control_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"

extern PR_TypeDef Current_PR_Loop_alpha;
extern float g_duty_a, g_duty_b, g_duty_c;

// Unified PI(voltage outer) + PR(current inner) control loop
void Task_Control_PI_PR_Loop(void)
{
    if (!Run_Flag) return;
    HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);

    static float i_mag = I_MAG_DEFAULT;
    static uint16_t v_dec = 0;
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
    static float uab_filt = 0.0f;
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
