#include "task_control_dcdc.h"
#include "task_pwm_dcdc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_dcdc.h"
#include "task_protect.h"
#include "task_display_dcdc.h"
#include "vofa.h"

// Buck: 电压外环 + 电流内环(P+I分离), ADC ISR @ 10kHz
void Task_Control_Buck(void)
{
    static uint8_t   prev_active = 0;
    static uint16_t  v_cnt = 0;
    static PI_TypeDef v_pi, i_pi;
    static float     vref  = 0.0f;
    static float     iref  = 0.0f;
    static float     il_f  = 0.0f;

    if (!Run_Flag) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active)
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        prev_active = 0;
        v_cnt = 0; vref = 0.0f; iref = 0.0f; il_f = 0.0f;
        return;
    }

    float vin  = U_line[2];
    float vout = U_line[3];
    uint8_t active = (vin > BUCK_VIN_MIN);
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    }
    prev_active = active;
    if (!active) return;

    if (rising) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        f32_PI_Init(&v_pi, 0.001f, BUCK_V_KP, BUCK_V_KI, (int16_t)BUCK_IL_MAX, 0);
        f32_PI_Init(&i_pi, 0.0001f, 0.0f, BUCK_I_KI, 32767, -32768);
        v_pi.x0 = v_pi.x1 = v_pi.y0 = v_pi.y1 = 0.0f;
        i_pi.x0 = i_pi.x1 = i_pi.y0 = i_pi.y1 = 0.0f;
        vref = 0.0f; iref = 0.0f; il_f = 0.0f; v_cnt = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R2_OV | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV;
        g_display_fn      = Task_Display_Buck;
        g_vofa_fn         = vofa_capture_buck;
    }

    // 电压外环 @ 1kHz
    if (++v_cnt >= 10) {
        v_cnt = 0;
        if (vref < BUCK_UREF) { vref += 0.1f; if (vref > BUCK_UREF) vref = BUCK_UREF; }
        float tmp = f32_PI_Calculate(&v_pi, vref, vout);
        if (tmp > BUCK_IL_MAX) { tmp = BUCK_IL_MAX; v_pi.y1 = BUCK_IL_MAX; }
        if (tmp < 0.0f)       { tmp = 0.0f;        v_pi.y1 = 0.0f; }
        iref = tmp;
    }

    // 电流内环 @ 10kHz (P+I 分离)
    float il_raw = -I_line[BUCK_ADC_IL_IDX];
    il_f += 0.02f * (il_raw - il_f);
    float error = iref - il_f;
    float duty = f32_PI_Calculate(&i_pi, iref, il_f) + BUCK_I_KP * error;
    if (duty > BUCK_DUTY_MAX) duty = BUCK_DUTY_MAX;
    if (duty < 0.0f)          duty = 0.0f;

    g_dbg_err   = iref;
    g_dbg_vctrl = duty;

    _pwm_buck(duty);
}

// ---- 固定占空比开环调试 ----
void Task_Debug_Buck(void)
{
    static uint8_t started = 0;

    if (!Run_Flag) {
        if (started) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
            started = 0;
        }
        return;
    }

    if (!started) {
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R2_OV | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        g_display_fn = Task_Display_Buck;
        g_vofa_fn    = vofa_capture_buck;
        started = 1;
    }

    _pwm_buck(BUCK_DEBUG_DUTY);
}
