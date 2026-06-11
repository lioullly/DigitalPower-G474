#include "task_control_pfc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def.h"
#include "task_pwm_1p.h"

/*
 * PFC_PHASE_DLY 相位-PF 对照表 (@10kHz, 50Hz电网, 1sample=1.8°)
 * ┌──────────┬───────────────┬──────┬──────────┐
 * │  延时    │      φ        │  PF  │   类型   │
 * ├──────────┼───────────────┼──────┼──────────┤
 * │    0     │      0°       │  1.0 │  UPF     │
 * │   17     │    30° lag    │ 0.87 │  感性    │
 * │   33     │    60° lag    │ 0.50 │  感性    │
 * │   50     │    90° lag    │ 0.00 │ 纯感性   │
 * │  100     │   180°(反相)  │ -1.0 │  逆变    │
 * │  150     │  270°=90°lead │ 0.00 │ 纯容性   │
 * │  167     │ 300°=60°lead  │-0.50 │  容性    │
 * │  183     │ 330°=30°lead  │-0.87 │  容性    │
 * │  200     │ 360°=  0°     │ 1.00 │  UPF     │
 * └──────────┴───────────────┴──────┴──────────┘
 * 感/容 = 感性PF为正 容性PF为负
 */
#define PFC_IREF_MAX   1.4f       // max RMS current reference [A]
#define PFC_IREF_PK   (PFC_IREF_MAX * 1.414f)  // peak

#define PFC_PHASE_DLY  0          // delay [samples @10kHz]; 0=UPF, 167=PF=-0.5
#define PFC_DLY_BUF  200          // buffer size (covers full 360°)


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
        f32_PI_Init(&v_pi, 0.001f, 0.05f, 0.7f, 10, -1);
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

    // --- current reference from voltage loop output ---
    static float dly_buf[PFC_DLY_BUF] = {0};
    static uint16_t dly_idx = 0;
    float peak = g_uab_rms * 1.414f;
    if (peak < 1.0f) peak = 1.0f;
    float v_template = U_line[0] / peak;
    float tpl = (PFC_PHASE_DLY > 0)
        ? dly_buf[(dly_idx + PFC_DLY_BUF - PFC_PHASE_DLY) % PFC_DLY_BUF]
        : v_template;
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
