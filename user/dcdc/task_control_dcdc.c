#include "task_control_dcdc.h"
#include "task_pwm_dcdc.h"
#include "user.h"
#include "pi_pr_ctrl.h"
#include "hardware_def_dcdc.h"
#include "task_protect.h"
#include "task_display_dcdc.h"
#include "vofa.h"

// Buck: 电压外环 PI + 电流内环 PI + 前馈, ADC ISR @ 10kHz
// TA1/TA2 单半桥
void Task_Control_Buck(void)
{
    static uint8_t  prev_active = 0;
    static uint16_t v_cnt = 0;
    static PI_TypeDef v_pi, i_pi;
    static float    vref  = 0.0f;   // 软启动电压目标
    static float    iref  = 0.0f;   // 电流参考 (电压环输出)

    if (!Run_Flag) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);    // UCC21520 DISABLE=H → 关驱动
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        if (prev_active) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        }
        prev_active = 0;
        v_cnt = 0;
        vref  = 0.0f;
        iref  = 0.0f;
        return;
    }

    float vin  = U_line[2];
    float vout = U_line[3];
    float il   = -I_line[BUCK_ADC_IL_IDX];  // 传感器正方向为流入电感, 读数为负

    // --- 激活检测: Vin > 最低启动电压 ---
    uint8_t active = (vin > BUCK_VIN_MIN);
    uint8_t rising = (active && !prev_active);

    if (!active && prev_active) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);    // UCC21520 DISABLE=H
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    }
    prev_active = active;
    if (!active) return;

    // --- 上升沿一次性初始化 ---
    if (rising) {
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);  // UCC21520 DISABLE=L → 开驱动
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);

        // 电压外环: Ts=0.001s (1kHz), 输出限幅 [0, BUCK_IL_MAX] A
        f32_PI_Init(&v_pi, 0.001f,
            BUCK_V_KP, BUCK_V_KI,
            (int16_t)BUCK_IL_MAX, 0);

        // 电流内环: Ts=0.0001s (10kHz), 输出占空比, 可正可负
        f32_PI_Init(&i_pi, 0.0001f,
            BUCK_I_KP, BUCK_I_KI,
            32767, -32768);

        vref  = 0.0f;
        iref  = 0.0f;
        v_cnt = 0;

        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A);
        g_protect_mask    = PROT_IL1_OC | PROT_ADC2_R2_OV | PROT_ADC2_R3_OV | PROT_ADC2_R4_OV;
        g_protect_ac_mask = 0;
        g_display_fn      = Task_Display_Buck;
        g_vofa_fn         = vofa_capture_buck;
    }

    // --- 电压外环 @ 1kHz (10:1 降采样) ---
    if (++v_cnt >= 10) {
        v_cnt = 0;
        // 软启动: Vref 缓慢斜坡到目标值
        if (vref < BUCK_UREF) {
            vref += 0.1f;
            if (vref > BUCK_UREF) vref = BUCK_UREF;
        }
        float tmp = f32_PI_Calculate(&v_pi, vref, vout);
        if (tmp > BUCK_IL_MAX) tmp = BUCK_IL_MAX;
        if (tmp < 0.0f)       tmp = 0.0f;
        iref = tmp;
    }

    // --- 电流内环 @ 10kHz ---
    float duty_i = f32_PI_Calculate(&i_pi, iref, il);
    if (duty_i >  BUCK_DUTY_MAX) duty_i =  BUCK_DUTY_MAX;
    if (duty_i < -BUCK_DUTY_MAX) duty_i = -BUCK_DUTY_MAX; 

    // 前馈
    float duty_ff = (vin > 1.0f) ? (vout / vin) : 0.0f;
    float duty = duty_ff + duty_i;

    if (duty > BUCK_DUTY_MAX) duty = BUCK_DUTY_MAX;
    if (duty < 0.0f)          duty = 0.0f;

    g_dbg_err   = iref;    // debug: 电流参考
    g_dbg_vctrl = duty;    // debug: 最终占空比

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
            HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);   // UCC21520 DISABLE=H
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
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);  // UCC21520 DISABLE=L → 开驱动
        g_display_fn = Task_Display_Buck;
        g_vofa_fn    = vofa_capture_buck;
        started = 1;
    }

    _pwm_buck(BUCK_DEBUG_DUTY);
}
