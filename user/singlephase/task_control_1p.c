#include "task_control_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_1p.h"
#include "task_pwm_1p.h"
#include "task_protect.h"
#include "task_display_1p.h"
#include "vofa.h"

// ----- off-grid inverter: voltage PI + PR current loop -----
// Voltage PI @ 200Hz → iref, PR current loop → v_ctrl

void Task_Control_OffGrid(void)
{
    static uint8_t  prev_active = 0;
    static uint16_t v_cnt = 0;
    static uint16_t oc_delay = 0;
    static PI_TypeDef v_pi;
    static float    iref = 0.0f;

    if (!Run_Flag) {
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        }
        prev_active = 0;
        v_cnt = 0;
        g_dbg_iref = -2.0f;  // VOFA marker: stopped
        return;
    }

    // --- activation: DC bus must be present (with hysteresis) ---
    uint8_t active;
    if (prev_active)
        active = (U_line[1] > (OFFGRID_UDC_MIN * 0.8f));
    else
        active = (U_line[1] > OFFGRID_UDC_MIN);
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active) {
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    }
    prev_active = active;

    if (!active) {
        g_dbg_iref = -1.0f;  // VOFA marker: waiting for activation
        return;
    }

    // --- one-shot init ---
    if (rising) {
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        f32_PR_Init(&Current_PR_Loop_alpha, 2.0f, 100.0f, 50.0f, 20.0f, 20000.0f, PR_CTRL_CLAMP, -PR_CTRL_CLAMP);
        f32_PI_Init(&v_pi, 0.005f, 0.05f, 1.0f, OFFGRID_IREF_MAX, 0);
        iref = 0.0f;
        v_cnt = 0;
        oc_delay = 0;
        // Start without OC — enable after 50ms ramp
        g_protect_mask    = PROT_UAC_OV | PROT_UDC_OV | PROT_UDC_UV;
        g_display_fn      = Task_Display_1P;
        g_vofa_fn         = vofa_capture_1p;
    }

    // --- OC enable delay: 50ms after activation ---
    if (oc_delay < 500) {
        oc_delay++;
        if (oc_delay >= 500)
            g_protect_mask |= PROT_IL1_OC;
    }

    // --- voltage loop: decimate 10kHz→200Hz ---
    if (++v_cnt >= 50) {
        v_cnt = 0;
        float tmp = f32_PI_Calculate(&v_pi, OFFGRID_UREF, g_uab_rms);
        // Asymmetric ramp: slow up, fast down
        if (tmp > iref + 0.5f) tmp = iref + 0.5f;
        if (tmp > OFFGRID_IREF_MAX) tmp = OFFGRID_IREF_MAX;
        if (tmp < 0.0f) tmp = 0.0f;
        iref = tmp;
    }

    // --- current reference: internally generated sine ---
    float i_ref = iref * 1.414f * g_sin_wt;
    float i_pk  = OFFGRID_IREF_MAX * 1.414f;
    if (i_ref >  i_pk) i_ref =  i_pk;
    if (i_ref < -i_pk) i_ref = -i_pk;

    // --- PR current loop with voltage feedforward ---
    // v_ref = v_ff + v_ctrl.  v_ff clamped to avoid clipping at bus limit.
    float i_fb   = -I_line[0];
    float i_err  = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    float v_ff   = OFFGRID_UREF * 1.414f * g_sin_wt;
    float v_max  = U_line[1] * 0.95f;
    if (v_ff >  v_max) v_ff =  v_max;
    if (v_ff < -v_max) v_ff = -v_max;
    float v_ref  = v_ff + v_ctrl;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    float m = v_ref / udc;

    g_dbg_err      = i_err;
    g_dbg_vctrl    = v_ctrl;
    g_dbg_iref     = iref;
    g_dbg_iref_inst = i_ref;
    g_dbg_m        = m;

    _pwm_bipolar(m);
}

// ============================================================
// 单相 SPWM 开环调试: 固定调制比 M=0.3, 50Hz
// 用法: g_control_isr = Task_Debug_SPWM;
// ============================================================
void Task_Debug_SPWM(void)
{
    static uint8_t started = 0;

    if (!Run_Flag) {
        if (started) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
            started = 0;
        }
        g_dbg_iref = -2.0f;  // VOFA marker: stopped
        return;
    }

    if (!started) {
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        g_protect_mask = PROT_IL1_OC;
        started = 1;
    }

    float m = 0.3f * g_sin_wt;
    g_dbg_iref     = 1.0f;    // VOFA marker: running
    g_dbg_iref_inst = g_sin_wt;
    g_dbg_m        = m;
    g_dbg_err      = I_line[0];
    g_dbg_vctrl    = U_line[1];

    _pwm_bipolar(m);
}
