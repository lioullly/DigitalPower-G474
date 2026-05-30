#include "user.h"
#include "user_tasks.h"
#include "task_pwm_1p.h"
#include "task_protect_1p.h"
#include "dac.h"

uint8_t Run_Flag = 0;
PI_TypeDef Voltage_PI_Loop;
PR_TypeDef Current_PR_Loop_alpha;
Integral_TypeDef Sine_Phase_Integrator;
float U_line[4] = {0}, I_line[3] = {0};
float g_duty_a = 0.5f, g_duty_b = 0.5f, g_duty_c = 0.5f;
float g_uab_rms = 0.0f, I_mag = I_MAG_DEFAULT, g_irms = 0.0f;
float U_coefficient, current_const, K_coefficient;
float Iref_alpha = 0.0f, Iref_beta = 0.0f;
volatile int32_t g_il1, g_il2, g_il3;
volatile uint8_t g_adc_data_ready;
float g_dbg_err, g_dbg_vctrl, g_dbg_m;
volatile float g_sin_wt;
uint16_t adc2_voltage_buffer[4];
uint16_t adc1_injected_buffer[2];  // DMA from ADC1: [IL1, IL2]
static float v_integ = 0.0f;

void user_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

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
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);  // DMA in background, no ISR needed

    HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 4);  // 50kHz/5=10kHz ADC
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // ADC4 disabled - kills scheduler on bare board

    GPIO_InitTypeDef btn = {0};
    btn.Pin  = user_Pin;
    btn.Mode = GPIO_MODE_INPUT;
    btn.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(user_GPIO_Port, &btn);

    HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Red_GPIO_Port,   Red_Pin,   GPIO_PIN_RESET);

    HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 8, 0);  // lower than HRTIM(7), don't preempt PWM
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);  // RX floating → noise storm

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    f32_PR_Init(&Current_PR_Loop_alpha, 0.5f, 50.0f, 50.0f, 10.0f, 10000.0f, 40.0f, -40.0f);
    f32_PI_Init(&Voltage_PI_Loop, 1.0f/10000.0f, 0.25f, 12.0f, (int16_t)I_MAG_MAX, 0);
    f32_Integral_Init(&Sine_Phase_Integrator, 1.0f/10000.0f, 1.0f);
    Sine_Phase_Integrator.x1 = 50.0f;  // pre-charge, avoid half-step on first call

    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HRTIM1->sMasterRegs.MDIER |= HRTIM_MDIER_MCMP1IE;  // re-apply after CubeMX clobber

    UserTasks_Init();
}

void Task_ADC_Fetch(void)
{
    if (!g_adc_data_ready) return;
    g_adc_data_ready = 0;
    I_line[0] = (float)g_il1 * current_const;
    U_line[0] = (float)(adc2_voltage_buffer[0] - 2036) * UAB_VOLTAGE_CONST;//UAB
    U_line[1] = (float)(adc2_voltage_buffer[1] - 0) * VOLTAGE_CONST;  //UDC

    static float uab_buf[200], il1_buf[200];
    static uint16_t idx = 0;
    uab_buf[idx] = U_line[0];
    il1_buf[idx] = I_line[0];
    if (++idx >= 200) {
        idx = 0;
        arm_rms_f32(uab_buf, 200, &g_uab_rms);
        arm_rms_f32(il1_buf, 200, &g_irms);
    }
}

void Task_Button_Scan(void)
{
    static uint8_t last_Run_Flag = 0xFF, last_user = 1;
    HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin);  // 50Hz heartbeat

    if (last_Run_Flag == 0xFF) last_user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    uint8_t user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    if (last_user == GPIO_PIN_SET && user == GPIO_PIN_RESET) Run_Flag = !Run_Flag;
    last_user = user;

    if (Run_Flag != last_Run_Flag) {
        last_Run_Flag = Run_Flag;
        if (Run_Flag) {
            f32_PR_Init(&Current_PR_Loop_alpha, 2.0f, 20.0f, 50.0f, 10.0f, 10000.0f, 25.0f, -25.0f);
            v_integ = 0.0f;  // not accessible here!
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
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
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        }
    }
}

// ADC ISR — phase accumulator + DAC locked to 10kHz ADC trigger
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 2050;

    g_wt = f32_Integral_Calculate(&Sine_Phase_Integrator, 50.0f);
    g_sin_wt = arm_sin_f32(g_wt * 2.0f * PI);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(g_sin_wt * 2047 + 2048));

    g_adc_data_ready = 1;
}

