#include "user.h"
#include "user_tasks.h"
#include "task_adc.h"
#include "task_pwm_1p.h"
#include "task_protect_1p.h"

uint8_t Run_Flag = 0;
uint8_t g_grid_mode = 0;  // 0=off-grid, 1=grid-tied
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

    Task_ADC_Init();  // ADC calibration, DMA, injected start, DAC, integrator

    f32_PI_Init(&Voltage_PI_Loop, 0.02f, 0.25f, 12.0f, (int16_t)I_MAG_MAX, 0);

    GPIO_InitTypeDef btn = {0};
    btn.Pin  = user_Pin;
    btn.Mode = GPIO_MODE_INPUT;
    btn.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(user_GPIO_Port, &btn);

    HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Red_GPIO_Port,   Red_Pin,   GPIO_PIN_RESET);

    HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);

    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);  // RX not used, keep USART1 IRQ for TX TC chain

    // Switch USART1 TX DMA from circular to normal for ping-pong VOFA
    HAL_DMA_DeInit(&hdma_usart1_tx);
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HRTIM1->sMasterRegs.MDIER |= HRTIM_MDIER_MCMP1IE;  // re-apply after CubeMX clobber

    UserTasks_Init();
}

void Task_Button_Scan(void)
{
    static uint8_t last_Run_Flag = 0xFF, last_user = 1;
    HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin);  // 50Hz heartbeat

    if (last_Run_Flag == 0xFF) last_user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    uint8_t user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    if (last_user == GPIO_PIN_SET && user == GPIO_PIN_RESET) {
        if (!Run_Flag && g_fault_code != 0) {
            g_fault_code = 0;  // clear fault, don't start
        } else {
            Run_Flag = !Run_Flag;
        }
    }
    last_user = user;

    if (Run_Flag != last_Run_Flag) {
        last_Run_Flag = Run_Flag;
        if (!Run_Flag) {
            // HRTIM handled by state machine — Button_Scan only toggles Run_Flag
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        }
    }
}
