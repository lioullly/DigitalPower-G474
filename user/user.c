#include "user.h"
#include "user_tasks.h"
#include "task_pll_1p.h"
#include "task_control_1p.h"
#include "task_pwm_1p.h"
#include "task_protect_1p.h"

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

#define ADC_FREQ   50000   // HRTIM triggers ADC at 50kHz
#define CTRL_FREQ  10000   // control task call rate (10kHz, period=100us)

// --- Off-grid state ---
#define WT_PERIOD  (CTRL_FREQ / 50)             // 200 steps per 50Hz cycle
#define SOFTSTART_STEPS  (CTRL_FREQ * 3 / 10)   // 300ms ramp = 3000 steps
static uint32_t wt_counter = 0;
static uint16_t ramp_cnt = 0;

// --- Grid-tied decimation counters ---
#define PLL_DECIMATE  10      // PLL at CTRL_FREQ/10 = 1kHz
#define PI_DECIMATE   100     // PI  at CTRL_FREQ/100 = 100Hz
static uint16_t pll_counter = 0;
static uint16_t pi_counter  = 0;

// ============================================================
//  Task_Control_Debug — fixed modulation 0.8, relay always ON
// ============================================================
void Task_Control_Debug(void)
{
    Task_Protect_1P();
    if (!Run_Flag) {
        ramp_cnt = 0;
        return;
    }

    HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);

    if (!g_adc_data_ready) return;
    g_adc_data_ready = 0;

    // Soft-start ramp
    if (ramp_cnt < SOFTSTART_STEPS) ramp_cnt++;
    float mod = OFFGRID_MOD_INDEX * (float)ramp_cnt / (float)SOFTSTART_STEPS;

    // Free-running 50Hz phase
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

// ============================================================
//  Task_Control_OffGrid — PI voltage + PR current, relay always ON
// ============================================================
void Task_Control_OffGrid(void)
{
    Task_Protect_1P();
    if (!Run_Flag) return;

    HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);

    if (!g_adc_data_ready) return;
    g_adc_data_ready = 0;

    // Free-running 50Hz phase → g_wt for PR current loop
    g_wt = (float)wt_counter / (float)WT_PERIOD;
    if (++wt_counter >= WT_PERIOD) wt_counter = 0;

    // PI voltage loop (decimated to 100Hz)
    if (++pi_counter >= PI_DECIMATE) {
        pi_counter = 0;
        Task_PI_VoltageLoop_1P();
    }

    // PR current loop
    Task_PR_CurrentLoop_1P();

    Task_PWM_1P_Update();
}

// ============================================================
//  Task_Control_GridTied — PLL + PR current loop only
// ============================================================
void Task_Control_GridTied(void)
{
    Task_Protect_1P();
    if (!Run_Flag) return;

    if (!g_adc_data_ready) return;
    g_adc_data_ready = 0;

    // PLL (decimated to 1kHz) → g_wt
    if (++pll_counter >= PLL_DECIMATE) {
        pll_counter = 0;
        Task_PLL_1P_Process();
    }

    // PR current loop
    Task_PR_CurrentLoop_1P();

    Task_PWM_1P_Update();
}

static void Current_PR_Init(void)
{
    float Kp = 2.0f;
    float Kr = 100.0f;
    float f0 = 50.0f;
    float BW = 5.0f;
    float Fs = (float)CTRL_FREQ;
    int16_t TH = 1200;
    int16_t TL = -1200;

    f32_PR_Init(&Current_PR_Loop_alpha, Kp, Kr, f0, BW, Fs, TH, TL);
    f32_PR_Init(&Current_PR_Loop_beta, Kp, Kr, f0, BW, Fs, TH, TL);
}

void user_Init(void)
{
    Uref = 32.0f;
    U_coefficient = VOLTAGE_CONST;
    current_const = CURRENT_CONST;
    K_coefficient = K_CONST;

    f32_PI_Init(&Voltage_PI_Loop, 0.01f, 0.25f, 12.0f, 32767, 0);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_voltage_buffer, 4);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc4);

    Current_PR_Init();

    // Init LED states (active-low: SET=off, RESET=on)
    HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Red_GPIO_Port,   Red_Pin,   GPIO_PIN_SET);

    // TIM6 @ 1MHz (1us) for scheduler timebase
    // APB1 timer clock = 160MHz, PSC=15 → 10MHz, ARR=9 → 1MHz
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->PSC = 15;
    TIM6->ARR = 9;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 8);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;

    UserTasks_Init();
}

void user_Loop(void)
{
}

void Task_Button_Scan(void)
{
    static uint8_t last_Run_Flag = 0xFF;
    static uint8_t last_user = 1;
    uint8_t user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);

    // user_Pin: toggle Run_Flag
    if (last_user == GPIO_PIN_SET && user == GPIO_PIN_RESET)
        Run_Flag = !Run_Flag;
    last_user = user;

    // Start/stop HRTIM on Run_Flag edge
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

// --- ADC ISR: lightweight, only reads raw values ---
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

    if (ADC_Injected_Flag != 0x05) return;
    ADC_Injected_Flag = 0;

    g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 4096;
    g_il2 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - 4096;
    g_il3 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_1) - 4096;

    I_line[0] = (float)g_il1 * current_const;
    I_line[1] = (float)g_il2 * current_const;
    I_line[2] = (float)g_il3 * current_const;

    U_line[0] = (float)(adc2_voltage_buffer[0] - 2048) * U_coefficient;
    U_line[1] = (float)(adc2_voltage_buffer[1] - 2048) * U_coefficient;
    U_line[2] = (float)(adc2_voltage_buffer[2] - 2048) * U_coefficient;
    U_line[3] = (float)(adc2_voltage_buffer[3] - 2048) * U_coefficient;

    g_adc_data_ready = 1;
}

// --- TIM6 ISR: scheduler tick @ 1MHz ---
void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;
        Scheduler_Tick();
    }
}
