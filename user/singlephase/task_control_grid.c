#include "task_control_grid.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_1p.h"
#include "task_pwm_1p.h"
#include "task_protect.h"
#include "task_display_1p.h"
#include "vofa.h"

// ----- grid-tied inverter: Hilbert sync + active/reactive power -----
// Hilbert locks grid phase, PR current loop injects active/reactive current.
// i_ref = i_mag * [v_α/v_mag + tan(φ)·v_β/v_mag]

void Task_Control_Grid(void)
{
    static uint8_t  prev_active = 0;
    static Hilbert_TypeDef hilbert;
    static float    last_phase = 0.0f;
    static float    i_mag = 0.05f;

    if (!Run_Flag) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
            HAL_HRTIM_WaveformCounterStop(&hhrtim1,
                HRTIM_TIMERID_TIMER_B);
        }
        prev_active = 0;
        return;
    }

    // --- activation: grid must be present (with hysteresis) ---
    uint8_t active;
    if (prev_active) {
        active = (g_uab_rms > (GRID_UREF_MIN * 0.8f));
    } else {
        active = (g_uab_rms > GRID_UREF_MIN);
    }
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStop(&hhrtim1,
            HRTIM_TIMERID_TIMER_B);
    }
    prev_active = active;

    if (!active) return;

    // --- one-shot init ---
    if (rising) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        f32_PR_Init(&Current_PR_Loop_alpha, 4.0f, 10.0f, 50.0f, 10.0f, 10000.0f, PR_CTRL_CLAMP, -PR_CTRL_CLAMP);
        f32_Hilbert_Init(&hilbert, 50.0f, 10000.0f);
        i_mag = 0.05f;
        last_phase = 0.0f;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_B);
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R1_OV | PROT_ADC2_R2_OV | PROT_ADC2_R2_UV;
        g_display_fn      = Task_Display_1P;
        g_vofa_fn         = vofa_capture_1p;
    }

    // --- Hilbert → v_α/v_β + frequency tracking ---
    float v_alpha, v_beta;
    f32_Hilbert_Calculate(&hilbert, U_line[0], &v_alpha, &v_beta);
    float phase = atan2f(v_beta, v_alpha);
    float dphase = phase - last_phase;
    if (dphase >  PI) dphase -= 2.0f * PI;
    if (dphase < -PI) dphase += 2.0f * PI;
    last_phase = phase;
    float freq_raw = dphase * 1591.55f;            // 10k/2π ≈ 1591.55
    g_freq_est += 0.02f * (freq_raw - g_freq_est); // LPF ~2Hz

    // --- soft-start ramp i_mag: 0.05A → GRID_I_MAG_DEFAULT @ ~1A/s ---
    if (i_mag < GRID_I_MAG_DEFAULT)
        i_mag += 0.0001f;   // 0.0001 * 10000 = 1A/s
    if (i_mag > GRID_I_MAG_DEFAULT)
        i_mag = GRID_I_MAG_DEFAULT;

    // --- current reference: i_mag(RMS) × 1.414 → peak, matching off-grid convention ---
    float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (v_mag < 1.0f) v_mag = 1.0f;
    float phi_rad = g_grid_phi_deg * (PI / 180.0f);
    float i_ref = i_mag * 1.414f * (arm_cos_f32(phi_rad) * (v_alpha / v_mag)
                                  + arm_sin_f32(phi_rad) * (v_beta / v_mag));
    float i_pk = GRID_IREF_MAX * 1.414f;
    if (i_ref >  i_pk) i_ref =  i_pk;
    if (i_ref < -i_pk) i_ref = -i_pk;

    // --- PR current loop (inverter: current out of bridge) ---
    float i_fb   = -I_line[0];
    float i_err  = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    float v_ref  = v_alpha + v_ctrl;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    float m = v_ref / udc;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;

    _pwm_bipolar(m);
}
