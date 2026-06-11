#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"

/*
 * g_pfc_phase_deg — 实测标定 (电流环 ~7° 系统延迟)
 * 线性插值可用, 容性 PF 用 >180° 值
 * ┌──────────┬────────────────┐
 * │  实测PF  │  g_pfc_phase_deg │
 * ├──────────┼────────────────┤
 * │  0.992   │    0.0         │
 * │  0.921   │   30.0         │
 * │  0.538   │   60.0         │
 * │  0.496   │   60.92        │
 * └──────────┴────────────────┘
 * 60.75≈PF0.5, 299.25≈PF-0.5
 */
#define PFC_IREF_MAX   5.66f      // max RMS current [A], peak ~8A, < IL1_OC(10A)
#define PFC_IREF_PK   (PFC_IREF_MAX * 1.414f)  // peak

// g_pfc_phase_deg — runtime adjustable via KEY1 (PB13) short +/-1°, long toggle step
#define PFC_DLY_BUF    200        // 1 grid cycle @ 50Hz (20ms)
#define PFC_DEG_PER_SAMPLE  (360.0f * 50.0f / 10000.0f)  // 1.8°/sample


void Task_Control_PFC(void)
{
    static uint8_t  prev_active = 0;
    static uint16_t v_cnt = 0;        // decimation counter: 10kHz→1kHz voltage loop
    static PI_TypeDef v_pi;
    static float    iref = 0.05f;     // RMS current reference from voltage loop

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
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    }
    prev_active = active;

    if (!active) return;

    // --- one-shot init ---
    if (rising) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        f32_PR_Init(&Current_PR_Loop_alpha, 4.0f, 10.0f, 50.0f, 10.0f, 10000.0f, 10, -10);
        f32_PI_Init(&v_pi, 0.001f, 0.1f, 2.0f, 10, -1);
        iref = 0.05f;
        v_cnt = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    }

    // --- voltage loop: decimate 10kHz→1kHz (every 10th call) ---
    if (++v_cnt >= 10) {
        v_cnt = 0;
        float tmp = f32_PI_Calculate(&v_pi, PFC_UREF, U_line[1]);
        if (tmp > PFC_IREF_MAX) tmp = PFC_IREF_MAX;
        if (tmp < 0.05f) tmp = 0.05f;
        iref = tmp;
    }

    // --- current reference with phase-shiftable template ---
    static float dly_buf[PFC_DLY_BUF] = {0};
    static uint16_t dly_idx = 0;
    float peak = g_uab_rms * 1.414f;
    if (peak < 1.0f) peak = 1.0f;
    float v_template = U_line[0] / peak;

    // linear interpolation for sub-sample phase resolution (~0.1°)
    float tpl;
    if (g_pfc_phase_deg > 0.0f) {
        float dly = g_pfc_phase_deg / PFC_DEG_PER_SAMPLE;  // fractional sample delay
        uint16_t lo = (uint16_t)dly;
        float    fr = dly - (float)lo;
        uint16_t hi = (lo + 1 < PFC_DLY_BUF) ? lo + 1 : lo;
        uint16_t i_lo = (dly_idx + PFC_DLY_BUF - lo) % PFC_DLY_BUF;
        uint16_t i_hi = (dly_idx + PFC_DLY_BUF - hi) % PFC_DLY_BUF;
        tpl = dly_buf[i_lo] * (1.0f - fr) + dly_buf[i_hi] * fr;
    } else {
        tpl = v_template;
    }
    dly_buf[dly_idx] = v_template;
    dly_idx = (dly_idx + 1) % PFC_DLY_BUF;
    float i_ref = tpl * (iref * 1.414f);
    if (i_ref >  PFC_IREF_PK) i_ref =  PFC_IREF_PK;
    if (i_ref < -PFC_IREF_PK) i_ref = -PFC_IREF_PK;

    // --- pure P current loop ---
    float i_fb   = I_line[0];
    float i_err  = i_ref - i_fb;
    float v_ctrl = f32_PR_Calculate(&Current_PR_Loop_alpha, i_err);
    float v_ref  = U_line[0] - v_ctrl;

    float udc = U_line[1];
    if (udc < 1.0f) udc = 1.0f;
    float m = v_ref / udc;

    g_dbg_err   = i_err;
    g_dbg_vctrl = v_ctrl;
    g_dbg_m     = m;

    _pwm_bipolar(m);
}
