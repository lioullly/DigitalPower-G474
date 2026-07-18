#include "task_control.h"
#include "task_pll.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_3p.h"

extern PR_TypeDef Current_PR_Loop_alpha;
extern PR_TypeDef Current_PR_Loop_beta;
extern PI_TypeDef Voltage_PI_Loop;

extern float g_duty_a, g_duty_b, g_duty_c;
extern float Iref_alpha, Iref_beta;
extern float I_mag;

void Task_PR_CurrentLoop(void)
{
    if (!Run_Flag) return;

    float cos_wt = arm_cos_f32(g_wt * 2.0f * PI);
    float sin_wt = arm_sin_f32(g_wt * 2.0f * PI);

    Iref_alpha = I_mag * cos_wt;
    Iref_beta  = I_mag * sin_wt;

    float i_alpha, i_beta;
    Clarke_Transform(I_line[0], I_line[1], I_line[2], &i_alpha, &i_beta);

    float err_alpha = Iref_alpha - i_alpha;
    float err_beta  = Iref_beta  - i_beta;

    float u_alpha = f32_PR_Calculate(&Current_PR_Loop_alpha, err_alpha);
    float u_beta  = f32_PR_Calculate(&Current_PR_Loop_beta,  err_beta);

    float uca = -(U_line[1] + U_line[3]);
    float v_g_alpha, v_g_beta;
    Clarke_Transform(U_line[1], U_line[3], uca, &v_g_alpha, &v_g_beta);

    float v_ref_alpha = u_alpha + v_g_alpha;
    float v_ref_beta  = u_beta  + v_g_beta;

    float v_abc[3];
    Inverse_Clarke_Transform(v_ref_alpha, v_ref_beta, &v_abc[0], &v_abc[1], &v_abc[2]);

    float udc = U_line[0];
    if (udc < 1.0f) udc = 1.0f;

    g_duty_a = v_abc[0] / udc + 0.5f;
    g_duty_b = v_abc[1] / udc + 0.5f;
    g_duty_c = v_abc[2] / udc + 0.5f;

    if (g_duty_a > 0.95f) g_duty_a = 0.95f;
    if (g_duty_b > 0.95f) g_duty_b = 0.95f;
    if (g_duty_c > 0.95f) g_duty_c = 0.95f;
    if (g_duty_a < 0.05f) g_duty_a = 0.05f;
    if (g_duty_b < 0.05f) g_duty_b = 0.05f;
    if (g_duty_c < 0.05f) g_duty_c = 0.05f;
}

void Task_PI_VoltageLoop(void)
{
    if (!Run_Flag) return;

    if (U_line[0] < 1.0f) return;

    I_mag = f32_PI_Calculate(&Voltage_PI_Loop, UREF, U_line[0]);
    if (I_mag < 0.0f) I_mag = 0.0f;
}
