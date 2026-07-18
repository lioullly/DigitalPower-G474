#include "task_control_dcdc.h"
#include "task_pwm_dcdc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_dcdc.h"
#include "task_protect.h"
#include "task_display_dcdc.h"
#include "vofa.h"

// Buck: 纯电流内环 (电压外环未启用), ADC ISR @ 10kHz
// TA1/TA2 单半桥, adc2_voltage_buffer 直读 DC 电压（无偏置）
void Task_Control_Buck(void)
{
    static uint8_t  prev_active = 0;
    static PI_TypeDef i_pi;
    static float    iref  = 0.0f;   // 电流参考 (固定值 + 软启动斜坡)
    static float    i_max = 0.0f;   // 软启动电流上限

    if (!Run_Flag) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
            // 注意: 不停 Timer A counter, 它是 ADC 触发源
        }
        prev_active = 0;
        iref  = 0.0f;
        i_max = 0.0f;
        return;
    }

    float vin  = U_line[2];
    float vout = U_line[3];
    float il   = I_line[BUCK_ADC_IL_IDX];

    // --- 激活检测: Vin > 最低启动电压 ---
    uint8_t active = (vin > BUCK_VIN_MIN);
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        // 注意: 不停 Timer A counter, 它是 ADC 触发源
    }
    prev_active = active;
    if (!active) return;

    // --- 上升沿一次性初始化 ---
    if (rising) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);

        // 电流内环: Ts=0.0001s (10kHz), 输出为占空比调整量 ×1000
        f32_PI_Init(&i_pi, 0.0001f,
            BUCK_I_KP, BUCK_I_KI,
            (int16_t)( BUCK_DUTY_ADJ_MAX * 1000.0f),
            (int16_t)(-BUCK_DUTY_ADJ_MAX * 1000.0f));

        iref  = 0.0f;
        i_max = BUCK_IL_START;

        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A);
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R2_OV | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV;
        g_protect_ac_mask = 0;  // Buck 全部 DC
        g_display_fn      = Task_Display_Buck;
        g_vofa_fn         = vofa_capture_buck;
    }

    // --- 软启动: iref 和电流上限同步斜坡 ---
    if (iref < BUCK_IREF_OPEN) {
        iref += BUCK_IREF_SLEW;
        if (iref > BUCK_IREF_OPEN) iref = BUCK_IREF_OPEN;
    }
    if (i_max < BUCK_IL_MAX) {
        i_max += BUCK_IL_SLEW;
        if (i_max > BUCK_IL_MAX) i_max = BUCK_IL_MAX;
    }
    if (iref > i_max) iref = i_max;  // 电流上限约束

    // --- 电流内环 @ 10kHz ---
    float duty_adj = f32_PI_Calculate(&i_pi, iref, il) * 0.001f;  // 缩放到占空比

    // 前馈: duty_ff = Vout / Vin (稳态关系)
    float duty_ff = (vin > 1.0f) ? (vout / vin) : 0.0f;
    float duty = duty_ff + duty_adj;

    if (duty > BUCK_DUTY_MAX) duty = BUCK_DUTY_MAX;
    if (duty < 0.0f)          duty = 0.0f;

    g_dbg_err   = iref;   // debug: 电流参考
    g_dbg_vctrl = duty;   // debug: 最终占空比

    _pwm_buck(duty);
}

// ---- 固定占空比开环调试: BUCK_DEBUG_DUTY ----
void Task_Debug_Buck(void)
{
    static uint8_t started = 0;

    if (!Run_Flag) {
        if (started) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
            // 注意: 不停 Timer A counter, 它是 ADC 触发源
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
            started = 0;
        }
        return;
    }

    if (!started) {
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R2_OV | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV;
        g_protect_ac_mask = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        g_display_fn = Task_Display_Buck;
        g_vofa_fn    = vofa_capture_buck;
        started = 1;
    }

    _pwm_buck(BUCK_DEBUG_DUTY);
}
