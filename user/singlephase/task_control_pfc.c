#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"
#include "task_pll_1p.h"

extern PR_TypeDef Current_PR_Loop_alpha;
extern float g_duty_a, g_duty_b, g_duty_c;

#define PFC_UDC_MAX UDC_OV
#define PFC_IREF_MAX 2.0f

void Task_Control_PFC(void)
{
    if (!Run_Flag || !g_pll_locked) return;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;

    if (udc > PFC_UDC_MAX) {
        g_duty_a = 0.5f; g_duty_b = 0.5f; g_duty_c = 0.5f;
        Task_PWM_1P_Update();
        return;
    }

    // --- soft-start ---
    static float v_target = 0.0f;
    if (v_target < 1.0f) v_target = udc;
    v_target += 0.01f * (UREF - v_target);

    // --- outer voltage loop: update PR frequency + current magnitude ---
    static float i_mag = 0.1f;
    static uint16_t v_dec = 0;
    if (++v_dec >= 200) {
        v_dec = 0;
        extern float g_freq_est;
        f32_PR_Init(&Current_PR_Loop_alpha, 0.5f, 50.0f, g_freq_est, 10.0f, 10000.0f, (float)UDC_OV, -(float)UDC_OV);
        i_mag += 0.02f * (v_target - udc);
        if (i_mag > I_MAG_MAX) i_mag = I_MAG_MAX;
        if (i_mag < 0.0f)      i_mag = 0.0f;
    }

    // --- inner PR: α-axis current tracking ---
    float i_ref = i_mag * arm_sin_f32(g_wt * 2.0f * PI);  // clean sine, PLL-locked
    if (i_ref >  PFC_IREF_MAX) i_ref =  PFC_IREF_MAX;
    if (i_ref < -PFC_IREF_MAX) i_ref = -PFC_IREF_MAX;

    float i_fb  = I_line[0];
    float i_err = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);

    float m = v_ctrl / udc;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;
    g_dbg_m     = m;

    // --- totem-pole switching: PLL phase selects active leg ---
    float d = 0.5f + 0.5f * m;
    if (d > 0.95f) d = 0.95f;
    if (d < 0.05f) d = 0.05f;

    if (g_wt < 0.5f) {  // positive half: leg A active
        g_duty_a = d;
        g_duty_b = 0.0f;
    } else {            // negative half: leg B active
        g_duty_a = 0.0f;
        g_duty_b = d;
    }
    g_duty_c = 0.0f;

    Task_PWM_1P_Update();
}
