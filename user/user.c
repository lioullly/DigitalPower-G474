#include "user.h"
#include "user_tasks.h"
#include "task_pwm_1p.h"
#include "task_protect_1p.h"

uint8_t Run_Flag = 0;
float U_line[4] = {0}, I_line[3] = {0};
float g_duty_a = 0.5f, g_duty_b = 0.5f, g_duty_c = 0.5f;
float g_uab_rms = 0.0f, I_mag = I_MAG_DEFAULT;
float U_coefficient, current_const, K_coefficient;
float Iref_alpha = 0.0f, Iref_beta = 0.0f;
volatile int32_t g_il1, g_il2, g_il3;
volatile uint8_t g_adc_data_ready;
uint16_t adc2_voltage_buffer[4];

void user_Init(void)
{
    U_coefficient = VOLTAGE_CONST;
    current_const = CURRENT_CONST;
    K_coefficient = K_CONST;

    // Re-do DLL calibration with longer timeout (bare board may be slow)
    HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3);
    if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 500) != HAL_OK) {
        // DLL failed — blink Green fast
        while (1) {
            HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin);
            for (volatile uint32_t i = 0; i < 200000; i++);
        }
    }

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_voltage_buffer, 4);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // ADC4 disabled - kills scheduler on bare board

    GPIO_InitTypeDef btn = {0};
    btn.Pin  = user_Pin;
    btn.Mode = GPIO_MODE_INPUT;
    btn.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(user_GPIO_Port, &btn);

    HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Red_GPIO_Port,   Red_Pin,   GPIO_PIN_RESET);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);

    UserTasks_Init();
}

#define CTRL_FREQ 10000
#define WT_PERIOD (CTRL_FREQ / 50)
#define SOFTSTART_STEPS (CTRL_FREQ * 3 / 10)
static uint32_t wt_counter = 0;
static uint16_t ramp_cnt = 0;

void Task_Control_Debug(void)
{
    if (!Run_Flag) { ramp_cnt = 0; return; }
    HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
    if (!g_adc_data_ready) return;
    g_adc_data_ready = 0;

    I_line[0] = (float)g_il1 * current_const;
    I_line[1] = (float)g_il2 * current_const;
    U_line[0] = (float)(adc2_voltage_buffer[0] - 2048) * U_coefficient;
    U_line[1] = (float)(adc2_voltage_buffer[1] - 2048) * U_coefficient;

    if (ramp_cnt < SOFTSTART_STEPS) ramp_cnt++;
    float mod = OFFGRID_MOD_INDEX * (float)ramp_cnt / (float)SOFTSTART_STEPS;
    float wt = (float)wt_counter / (float)WT_PERIOD;
    if (++wt_counter >= WT_PERIOD) wt_counter = 0;
    float sin_wt = arm_sin_f32(wt * 2.0f * PI);

    g_duty_a = 0.5f + 0.5f * mod * sin_wt;
    g_duty_b = 0.5f - 0.5f * mod * sin_wt;
    if (g_duty_a > 0.95f) g_duty_a = 0.95f;
    if (g_duty_b > 0.95f) g_duty_b = 0.95f;
    if (g_duty_a < 0.05f) g_duty_a = 0.05f;
    if (g_duty_b < 0.05f) g_duty_b = 0.05f;
    g_duty_c = 0.5f;
    Task_PWM_1P_Update();
}

// Lightweight ADC callback - only raw reads, no float math
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 4096;
    g_il2 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - 4096;
    g_adc_data_ready = 1;
}

void user_Loop(void) {}

void Task_Button_Scan(void)
{
    static uint8_t last_Run_Flag = 0xFF, last_user = 1, hb = 0;
    HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin);  // 50Hz heartbeat

    if (last_Run_Flag == 0xFF) last_user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    uint8_t user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    if (last_user == GPIO_PIN_SET && user == GPIO_PIN_RESET) Run_Flag = !Run_Flag;
    last_user = user;

    if (Run_Flag != last_Run_Flag) {
        last_Run_Flag = Run_Flag;
        if (Run_Flag) {
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
            HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
            HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            HAL_HRTIM_WaveformCounterStart(&hhrtim1,
                HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_F);
        } else {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            HAL_HRTIM_WaveformCounterStop(&hhrtim1,
                HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_F);
            HAL_HRTIM_WaveformCounterStop(&hhrtim1, HRTIM_TIMERID_MASTER);
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        }
    }
}
