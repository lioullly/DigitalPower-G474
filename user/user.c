#include "user.h"
#include "user_tasks.h"

Integral_TypeDef Get_Sin;

PI_TypeDef Voltage_PI_Loop;
PR_TypeDef Current_PR_Loop_alpha;
PR_TypeDef Current_PR_Loop_beta;

uint8_t Run_Flag = 0;
volatile uint8_t ADC_Injected_Flag = 0;
volatile uint8_t g_adc_data_ready = 0;

uint16_t  adc2_voltage_buffer[4];

volatile int32_t g_il1 = 0;
volatile int32_t g_il2 = 0;
volatile int32_t g_il3 = 0;

float U_coefficient;
float current_const;
float K_coefficient;

float Uref;
float U_line[4];
float I_line[3];

float Iref_alpha = 0.0f;
float Iref_beta  = 0.0f;
float g_duty_a = 0.5f;
float g_duty_b = 0.5f;
float g_duty_c = 0.5f;

static void Current_PR_Init(void)
{
    float Kp = 10.0f;
    float Kr = 100.0f;
    float f0 = 50.0f;
    float BW = 5.0f;
    float Fs = 10000.0f;
    int16_t TH = 32767;
    int16_t TL = -32768;

    f32_PR_Init(&Current_PR_Loop_alpha, Kp, Kr, f0, BW, Fs, TH, TL);
    f32_PR_Init(&Current_PR_Loop_beta, Kp, Kr, f0, BW, Fs, TH, TL);
}

void user_Init(void)
{
    Uref = 32.0f;
    U_coefficient = VOLTAGE_CONST;
    current_const = CURRENT_CONST;
    K_coefficient = K_CONST;

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_voltage_buffer, 4);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc4);

    Current_PR_Init();

    UserTasks_Init();
}

void user_Loop(void)
{
}

void Task_Button_Scan(void)
{
    static uint8_t last_Run_Flag = 0xFF;
    static uint8_t last_button_state = 1;
    uint8_t current_state = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);

    if (last_button_state == GPIO_PIN_SET && current_state == GPIO_PIN_RESET)
    {
        Run_Flag = !Run_Flag;
    }
    last_button_state = current_state;

    if (Run_Flag != last_Run_Flag)
    {
        last_Run_Flag = Run_Flag;

        if (Run_Flag)
        {
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
            HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
            HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            HAL_HRTIM_WaveformCounterStart(&hhrtim1,
                HRTIM_TIMERID_TIMER_A |
                HRTIM_TIMERID_TIMER_B |
                HRTIM_TIMERID_TIMER_F);
        }
        else
        {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            HAL_HRTIM_WaveformCounterStop(&hhrtim1,
                HRTIM_TIMERID_TIMER_A |
                HRTIM_TIMERID_TIMER_B |
                HRTIM_TIMERID_TIMER_F);
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        }
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Injected_Flag |= 0x01;
    }
    else if (hadc->Instance == ADC4)
    {
        ADC_Injected_Flag |= 0x04;
    }
    else
    {
        return;
    }

    if (ADC_Injected_Flag == 0x05)
    {
        ADC_Injected_Flag = 0;

        g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 4095;
        g_il2 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - 4095;
        g_il3 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_1) - 4095;

        g_adc_data_ready = 1;
    }
}
