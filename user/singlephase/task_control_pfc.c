#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_1p.h"
#include "task_pwm_1p.h"
#include "task_protect.h"
#include "task_display_1p.h"
#include "vofa.h"


void Task_Control_PFC(void)
{
    static uint8_t  prev_active = 0;
    static uint16_t v_cnt = 0;
    static uint16_t oc_delay = 0;
    static PI_TypeDef v_pi;
    static Hilbert_TypeDef hilbert;
    static Notch_TypeDef udc_notch;
    static float    iref = 0.05f;

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

    // --- activation with hysteresis ---
    // First start: Udc > Vac_rms (allows starting from unregulated rectifier)
    // Keep alive:  Udc > Vac_rms * 0.7
    uint8_t active;
    if (prev_active) {
        active = (g_uab_rms > 3.0f)
              && (U_line[1] > (g_uab_rms * 0.7f));
    } else {
        active = (g_uab_rms > 5.0f)
              && (U_line[1] > (g_uab_rms * 1.0f));
    }
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
        f32_PR_Init(&Current_PR_Loop_alpha, 3.0f, 15.0f, 50.0f, 15.0f, 20000.0f, PR_CTRL_CLAMP, -PR_CTRL_CLAMP);
        f32_PI_Init(&v_pi, 0.001f, 0.1f, 0.5f, PFC_IREF_MAX, -1.0f);
        f32_Hilbert_Init(&hilbert, 50.0f, 20000.0f);
        f32_Notch_Init(&udc_notch, 100.0f, 5.0f, 1000.0f);
        // Prime notch filter with current Udc to avoid startup transient
        udc_notch.x1 = udc_notch.x2 = udc_notch.y1 = udc_notch.y2 = U_line[1];
        iref = 0.05f;
        v_cnt = 0;
        oc_delay = 0;
        // Start without OC — enable after 50ms ramp
        g_protect_mask    = PROT_UAC_OV | PROT_UDC_OV | PROT_UDC_UV;
        g_display_fn      = Task_Display_1P;
        g_vofa_fn         = vofa_capture_1p;
    }

    // --- OC enable delay: 50ms after PFC activation ---
    if (oc_delay < 1000) {  // 50ms @ 20kHz
        oc_delay++;
        if (oc_delay >= 1000)
            g_protect_mask |= PROT_IL1_OC;
    }

    // --- voltage loop: decimate 10kHz→1kHz (every 10th call) ---
    if (++v_cnt >= 20) {  // 20:1 @ 20kHz → 1kHz
        v_cnt = 0;
        float udc_filt = f32_Notch_Calculate(&udc_notch, U_line[1]);
        // Pure PI voltage loop (FF removed for baseline verification)
        float tmp = f32_PI_Calculate(&v_pi, PFC_UREF, udc_filt);
        // Virtual impedance: reduce current ceiling when Udc overshoots
        float iref_ceil = PFC_IREF_MAX;
        if (udc_filt > PFC_UREF)
            iref_ceil = PFC_IREF_MAX - (udc_filt - PFC_UREF) * 0.5f;
        if (iref_ceil < 0.0f) iref_ceil = 0.0f;
        if (tmp > iref_ceil) tmp = iref_ceil;
        if (tmp < 0.0f) tmp = 0.0f;
        iref = tmp;
    }

    // --- Hilbert phase-locked current reference ---
    // Phase from Hilbert (tracks AC zero-crossing), amplitude from iref (clean sine)
    float v_alpha, v_beta;
    f32_Hilbert_Calculate(&hilbert, U_line[0], &v_alpha, &v_beta);
    float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (v_mag < 1.0f) v_mag = 1.0f;
    float tpl = v_alpha / v_mag;        // unit sine, phase-locked to AC
    float phi_rad = g_pfc_phase_deg * (PI / 180.0f);
    if (phi_rad > 0.01f || phi_rad < -0.01f) {
        v_beta = v_beta / v_mag;        // unit cosine
        tpl = arm_cos_f32(phi_rad) * (v_alpha / v_mag)
            + arm_sin_f32(phi_rad) * (v_beta);
    }
    float i_ref = tpl * (iref * 1.414f);

    // ---  PR current loop ---
    float i_fb   = I_line[0];
    float i_err  = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    float v_ref  = v_alpha - v_ctrl;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    float m = v_ref / udc;

    g_dbg_err      = i_err;
    g_dbg_vctrl    = v_ctrl;
    g_dbg_iref     = iref;
    g_dbg_iref_inst = i_ref;
    g_dbg_m        = m;

    Task_PWM_1P_UniUpdate(m, v_alpha);
}
