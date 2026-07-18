#include "user.h"
#include "user_tasks.h"
#include "task_adc.h"
#include "task_pwm_1p.h"
#include "task_protect.h"
#include "timebase_scheduler.h"
#include "ssd1306.h"

uint8_t Run_Flag = 0;
PI_TypeDef Voltage_PI_Loop;
PR_TypeDef Current_PR_Loop_alpha;
Integral_TypeDef Sine_Phase_Integrator;
float U_line[4] = {0}, I_line[3] = {0};
float g_duty_a = 0.5f, g_duty_b = 0.5f, g_duty_c = 0.5f;
float g_uab_rms = 0.0f, I_mag = 1.0f, g_irms = 0.0f;
float U_coefficient, current_const;
float Iref_alpha = 0.0f, Iref_beta = 0.0f;
volatile int32_t g_il1, g_il2, g_il3;
volatile uint8_t g_adc_data_ready;
float g_dbg_err, g_dbg_vctrl, g_isr_khz;
float g_pfc_phase_deg = 0.0f;
uint8_t g_oled_ok = 0;
float g_grid_phi_deg  = 0.0f;
float   g_wt         = 0.0f;
uint8_t g_pll_locked = 0;
float   g_freq_est   = 50.0f;
volatile float g_sin_wt;
uint16_t adc2_voltage_buffer[4];
void (*g_adc_preproc)(void) = NULL;
void (*g_display_fn)(void)  = NULL;
void (*g_vofa_fn)(void)     = NULL;

void user_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    U_coefficient = VOLTAGE_CONST;
    current_const = CURRENT_CONST;

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

    f32_PI_Init(&Voltage_PI_Loop, 0.02f, 0.25f, 12.0f, 10, 0);

    // --- OLED init (non-critical, skip if not connected) ---
    if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 2, 10) == HAL_OK) {
        ssd1306_Init();
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        g_oled_ok = 1;
    }

    HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Red_GPIO_Port,   Red_Pin,   GPIO_PIN_RESET);

    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);  // RX not used, keep USART1 IRQ for TX TC chain

    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);  // ADC trigger source
    HRTIM1->sMasterRegs.MDIER |= HRTIM_MDIER_MCMP1IE;  // re-apply after CubeMX clobber

    UserTasks_Init();
}

void Task_Button_Scan(void)
{
    // --- heartbeat ---
    HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin);  // 50Hz

    // --- PC5 (user): Run_Flag toggle ---
    static uint8_t last_Run_Flag = 0xFF, last_user = 1;
    if (last_Run_Flag == 0xFF) last_user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    uint8_t user = HAL_GPIO_ReadPin(user_GPIO_Port, user_Pin);
    if (last_user == GPIO_PIN_SET && user == GPIO_PIN_RESET) {
        if (!Run_Flag) {
            g_fault_code = 0;   // clear any latched fault
            Run_Flag = 1;       // start
        } else {
            Run_Flag = 0;       // stop
        }
    }
    last_user = user;

    if (Run_Flag != last_Run_Flag) {
        last_Run_Flag = Run_Flag;
        if (!Run_Flag) {
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
        }
    }

    // --- KEY1(-) / KEY2(+): phase adjust, common step, long press toggles coarse/fine ---
    static uint16_t k1_hold = 0, k2_hold = 0;
    static uint8_t  k1_last = 1, k2_last = 1;
    static float    ph_step = 1.0f;
    uint8_t k1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    uint8_t k2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);

    // KEY1: decrement
    if (k1_last == 1 && k1 == 0) k1_hold = 0;
    else if (k1 == 0) k1_hold++;
    else if (k1_last == 0 && k1 == 1) {
        if (k1_hold < 50) {
            g_pfc_phase_deg -= ph_step; if (g_pfc_phase_deg < 0.0f) g_pfc_phase_deg += 360.0f;
            g_grid_phi_deg  -= ph_step; if (g_grid_phi_deg  < 0.0f) g_grid_phi_deg  += 360.0f;
        } else ph_step = (ph_step > 0.5f) ? 0.1f : 1.0f;
    }
    k1_last = k1;

    // KEY2: increment
    if (k2_last == 1 && k2 == 0) k2_hold = 0;
    else if (k2 == 0) k2_hold++;
    else if (k2_last == 0 && k2 == 1) {
        if (k2_hold < 50) {
            g_pfc_phase_deg += ph_step; if (g_pfc_phase_deg >= 360.0f) g_pfc_phase_deg -= 360.0f;
            g_grid_phi_deg  += ph_step; if (g_grid_phi_deg  >= 360.0f) g_grid_phi_deg  -= 360.0f;
        } else ph_step = (ph_step > 0.5f) ? 0.1f : 1.0f;
    }
    k2_last = k2;
}
