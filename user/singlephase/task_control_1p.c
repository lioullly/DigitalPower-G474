#include "task_control_1p.h"
#include "task_pll_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"

extern PR_TypeDef Current_PR_Loop_alpha;
extern PI_TypeDef Voltage_PI_Loop;

extern float g_duty_a, g_duty_b, g_duty_c;

void Task_PR_CurrentLoop_1P(void)
{
    if (!Run_Flag) return;

    float i_ref = I_mag * 1.414214f * g_sin_wt;  // RMS→peak, use g_sin_wt
    float i_fb  = -I_line[0];                     // sensor inverted
    float err   = i_ref - i_fb;

    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, err);

    static float uab_filt = 0.0f;
    uab_filt += 0.1f * (U_line[0] - uab_filt);  // ~160Hz LPF
    float v_ref = v_ctrl + uab_filt;              // Uab feedforward

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;

    float m = v_ref / udc;

    g_duty_a = 0.5f + 0.5f * m;
    g_duty_b = 0.5f - 0.5f * m;

    if (g_duty_a > 0.95f) g_duty_a = 0.95f;
    if (g_duty_b > 0.95f) g_duty_b = 0.95f;
    if (g_duty_a < 0.05f) g_duty_a = 0.05f;
    if (g_duty_b < 0.05f) g_duty_b = 0.05f;

    g_duty_c = 0.5f;
}

void Task_PI_VoltageLoop_1P(void)
{
    if (!Run_Flag) return;

    if (g_uab_rms < 1.0f) return;

    I_mag = f32_PI_Calculate(&Voltage_PI_Loop, UREF, g_uab_rms);
    if (I_mag < 0.0f) I_mag = 0.0f;
}
