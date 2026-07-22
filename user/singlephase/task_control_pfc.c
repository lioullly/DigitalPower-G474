#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_1p.h"
#include "task_pwm_1p.h"
#include "task_protect.h"
#include "task_display_1p.h"
#include "vofa.h"

#define PFC_IREF_PK   (PFC_IREF_MAX * 1.414f)  // peak


void Task_Control_PFC(void)
{
    static uint8_t  prev_active = 0;
    static uint16_t v_cnt = 0;
    static PI_TypeDef v_pi;
    static Hilbert_TypeDef hilbert;
    static Notch_TypeDef udc_notch;
    static float    iref = 0.05f;

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
        v_cnt = 0;
        return;
    }

    // --- activation with hysteresis ---
    uint8_t active;
    if (prev_active) {
        active = (g_uab_rms > 3.0f)
              && (U_line[1] > (g_uab_rms * 0.8f));
    } else {
        active = (g_uab_rms > 5.0f)
              && (U_line[1] > (g_uab_rms * 1.2f));
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
        f32_PI_Init(&v_pi, 0.001f, 0.1f, 2.0f, 10, -1);
        f32_Hilbert_Init(&hilbert, 50.0f, 10000.0f);
        f32_Notch_Init(&udc_notch, 100.0f, 1.0f, 1000.0f);
        iref = 0.05f;
        v_cnt = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_B);
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R1_OV | PROT_ADC2_R2_OV | PROT_ADC2_R2_UV;
        g_display_fn      = Task_Display_1P;
        g_vofa_fn         = vofa_capture_1p;
    }

    // --- voltage loop: decimate 10kHz→1kHz (every 10th call) ---
    if (++v_cnt >= 10) {
        v_cnt = 0;
        float udc_filt = f32_Notch_Calculate(&udc_notch, U_line[1]);
        float tmp = f32_PI_Calculate(&v_pi, PFC_UREF, udc_filt);
        if (tmp > PFC_IREF_MAX) tmp = PFC_IREF_MAX;
        if (tmp < 0.05f) tmp = 0.05f;
        iref = tmp;
    }

    // --- Hilbert phase-shifted template (replaces delay buffer) ---
    float v_alpha, v_beta;
    f32_Hilbert_Calculate(&hilbert, U_line[0], &v_alpha, &v_beta);
    float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (v_mag < 1.0f) v_mag = 1.0f;
    // tpl = cos(φ)·v_α/v_mag + sin(φ)·v_β/v_mag
    float phi_rad = g_pfc_phase_deg * (PI / 180.0f);
    float tpl = arm_cos_f32(phi_rad) * (v_alpha / v_mag)
              + arm_sin_f32(phi_rad) * (v_beta / v_mag);
    float i_ref = tpl * (iref * 1.414f);
    if (i_ref >  PFC_IREF_PK) i_ref =  PFC_IREF_PK;
    if (i_ref < -PFC_IREF_PK) i_ref = -PFC_IREF_PK;

    // ---  PR current loop ---
    float i_fb   = I_line[0];
    float i_err  = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    float v_ref  = v_alpha - v_ctrl;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    float m = v_ref / udc;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;

    _pwm_bipolar(m);
}
