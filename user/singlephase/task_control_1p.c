#include "task_control_1p.h"
#include "task_pll_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"

extern PR_TypeDef Current_PR_Loop_alpha;
extern PI_TypeDef Voltage_PI_Loop;

extern float g_duty_a, g_duty_b, g_duty_c;
extern float I_mag;

void Task_PR_CurrentLoop_1P(void)
{
    if (!Run_Flag) return;

    float cos_wt = arm_cos_f32(g_wt * 2.0f * PI);
    float i_ref = I_mag * cos_wt;

    float err = i_ref - I_line[0];

    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, err);

    float v_ref = v_ctrl + U_line[1];

    float udc = U_line[0];
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

    if (U_line[0] < 1.0f) return;

    I_mag = f32_PI_Calculate(&Voltage_PI_Loop, UREF, U_line[0]);
    if (I_mag < 0.0f) I_mag = 0.0f;
}
