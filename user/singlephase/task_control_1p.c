#include "task_control_1p.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"

// ----- off-grid inverter: voltage PI + PR current loop -----
// Voltage PI @ 1kHz → iref, PR current loop → v_ctrl

void Task_Control_OffGrid(void)
{
    static uint8_t  prev_active = 0;
    static uint16_t v_cnt = 0;
    static PI_TypeDef v_pi;
    static float    iref = 0.0f;

    if (!Run_Flag) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
            HAL_HRTIM_WaveformCounterStop(&hhrtim1,
                HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
        }
        prev_active = 0;
        v_cnt = 0;
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
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStop(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    }
    prev_active = active;

    if (!active) return;

    // --- one-shot init ---
    static float soft_max = 0.1f;  // soft-start ramp ceiling

    if (rising) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        f32_PR_Init(&Current_PR_Loop_alpha, 4.0f, 10.0f, 50.0f, 10.0f, 25000.0f, PR_CTRL_CLAMP, -PR_CTRL_CLAMP);
        f32_PI_Init(&v_pi, 0.005f, 0.1f, 0.5f, OFFGRID_IREF_MAX, 0);
        soft_max = 0.1f;
        iref = 0.0f;
        v_cnt = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    }

    // --- voltage loop: decimate 25kHz→200Hz ---
    if (++v_cnt >= 125) {
        v_cnt = 0;
        // soft-start ramp: 0.1A → OFFGRID_IREF_MAX over ~1s
        if (soft_max < OFFGRID_IREF_MAX)
            soft_max += 0.025f;  // +0.025A/step @ 200Hz = +5A/s
        float tmp = f32_PI_Calculate(&v_pi, OFFGRID_UREF, g_uab_rms);
        if (tmp >  soft_max) tmp =  soft_max;
        if (tmp < 0.0f)      tmp = 0.0f;
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

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;

    _pwm_bipolar(m);
}
