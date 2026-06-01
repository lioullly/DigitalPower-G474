#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"
#include "task_pll_1p.h"

extern PR_TypeDef Current_PR_Loop_alpha;
extern float g_duty_a, g_duty_b, g_duty_c;
extern float g_freq_est;

#define PFC_UDC_MAX DC_OV
#define PFC_IREF_MAX 2.0f

void Task_Control_PFC(void)
{
    // --- edge detection: must be before early return to catch falling edge ---
    static uint8_t prev_active = 0;
    uint8_t active = (Run_Flag && g_pll_locked);
    uint8_t rising = (active && !prev_active);
    prev_active = active;

    if (!active) return;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;

    if (udc > PFC_UDC_MAX) {
        g_duty_a = 0.5f; g_duty_b = 0.5f; g_duty_c = 0.5f;
        Task_PWM_1P_Update();
        return;
    }

    // --- one-shot init on activation rising edge ---
    static float i_mag = 0.1f;
    static uint16_t v_dec = 0;

    if (rising) {
        f32_PR_Init(&Current_PR_Loop_alpha, 0.5f, 50.0f, g_freq_est, 20.0f, 10000.0f, (float)DC_OV, -(float)DC_OV);
        i_mag = 0.1f;
        v_dec = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_F);
    }

    // --- voltage outer loop, every 20ms ---
    if (++v_dec >= 200) {
        v_dec = 0;
        i_mag += 0.02f * (UREF - udc);
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

    // --- totem-pole with zero-crossing dead band ---
    // active leg: PWM (d clamped 0.05~0.95)
    // inactive leg: cmp=0 → high OFF, low ON (synchronous rectifier return path)
    // dead zone: both cmp=0 → both high OFF, both low ON (freewheel)
    #define ZC_DEADBAND  0.011f  // 0.07 rad

    float d = m;
    if (d > 0.95f) d = 0.95f;
    if (d < 0.05f) d = 0.05f;

    float wt = g_wt;
    uint8_t in_dz = (wt < ZC_DEADBAND || wt > (1.0f - ZC_DEADBAND) ||
                     (wt > (0.5f - ZC_DEADBAND) && wt < (0.5f + ZC_DEADBAND)));

    uint32_t period = 51200;
    uint32_t cmp_a, cmp_b;

    if (in_dz) {
        cmp_a = 0;
        cmp_b = 0;
    } else if (wt < 0.5f) {
        cmp_a = (uint32_t)(d * (float)period);
        cmp_b = 0;
    } else {
        cmp_a = 0;
        cmp_b = (uint32_t)(d * (float)period);
    }

    if (cmp_a > period) cmp_a = period;
    if (cmp_b > period) cmp_b = period;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}
