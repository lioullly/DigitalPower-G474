#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"
#include "task_pll_1p.h"

#define PFC_UDC_MAX    DC_OV
#define PFC_IREF_MAX   2.0f
#define PFC_VDC_MIN    (UREF * 0.8f)
#define ZC_DEADBAND    0.011f   // ~0.07 rad
#define PWM_PERIOD     51200U

void Task_Control_PFC(void)
{
    // --- edge detection ---
    static uint8_t prev_active = 0;
    uint8_t active = (Run_Flag && g_pll_locked && U_line[1] > PFC_VDC_MIN);
    uint8_t rising = (active && !prev_active);
    prev_active = active;

    if (!active) return;

    float udc_raw = U_line[1];

    // --- 100Hz notch on Udc (2ω ripple), fs=10kHz, bilinear from (s²+628²)/(s²+50s+628²) ---
    static float nx1 = 0, nx2 = 0, ny1 = 0, ny2 = 0;
    float udc_filt = 0.997509f*udc_raw - 1.991086f*nx1 + 0.997509f*nx2
                                       + 1.991086f*ny1 - 0.995017f*ny2;
    nx2 = nx1; nx1 = udc_raw;
    ny2 = ny1; ny1 = udc_filt;
    if (udc_filt < 1.0f) udc_filt = 1.0f;

    if (udc_filt > PFC_UDC_MAX) {
        g_duty_a = 0.5f; g_duty_b = 0.5f;
        Task_PWM_1P_Update();
        return;
    }

    // --- one-shot init on activation rising edge ---
    static float i_mag = 0.1f;
    static uint16_t v_dec = 0;

    if (rising) {
        f32_PR_Init(&Current_PR_Loop_alpha, 0.5f, 5.0f, g_freq_est, 20.0f, 10000.0f, (float)DC_OV, -(float)DC_OV);
        i_mag = 0.1f;
        v_dec = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    }

    // --- voltage outer loop, every 20ms ---
    if (++v_dec >= 200) {
        v_dec = 0;
        i_mag = f32_PI_Calculate(&Voltage_PI_Loop, UREF, udc_filt);
    }

    // --- inner PR: α-axis current tracking ---
    float i_ref = i_mag * arm_cos_f32(g_wt * 2.0f * PI);  // cos(θ_pll) = in-phase with v_α
    if (i_ref >  PFC_IREF_MAX) i_ref =  PFC_IREF_MAX;
    if (i_ref < -PFC_IREF_MAX) i_ref = -PFC_IREF_MAX;

    float i_fb   = I_line[0];
    float i_err  = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    float v_ff   = U_line[0];
    float v_ref  = v_ff - v_ctrl;
    float m      = v_ref / udc_filt;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;
    g_dbg_m     = m;

    // --- totem-pole PWM with zero-crossing dead band ---
    float d = m;
    if (d > 0.95f) d = 0.95f;
    if (d < 0.05f) d = 0.05f;

    float wt = g_wt;
    uint8_t in_dz = (wt < ZC_DEADBAND || wt > (1.0f - ZC_DEADBAND) ||
                     (wt > (0.5f - ZC_DEADBAND) && wt < (0.5f + ZC_DEADBAND)));

    uint32_t cmp_a, cmp_b;

    if (in_dz) {
        cmp_a = 0;
        cmp_b = 0;
    } else if (wt < 0.5f) {
        cmp_a = (uint32_t)(d * (float)PWM_PERIOD);
        cmp_b = 0;
    } else {
        cmp_a = 0;
        cmp_b = (uint32_t)(d * (float)PWM_PERIOD);
    }

    if (cmp_a > PWM_PERIOD) cmp_a = PWM_PERIOD;
    if (cmp_b > PWM_PERIOD) cmp_b = PWM_PERIOD;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}
