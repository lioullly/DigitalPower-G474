#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "task_adc.h"
#include "task_protect.h"
#include "vofa.h"
#include "task_pwm_1p.h"
#include "task_control_1p.h"
#include "task_control_pfc.h"
#include "task_control_grid.h"
#include "task_control_dcdc.h"
#include "task_display_1p.h"
#include "task_display_dcdc.h"
#include "dac.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

// ---- open-loop debug: fixed m=0.3, 50Hz sine from g_sin_wt ----
void Task_Debug_SPWM(void)
{
    static uint8_t started = 0;

    if (!Run_Flag) {
        if (started) {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
            // 注意: 不停 Timer A counter, 它是 ADC 触发源
            HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_RESET);
            started = 0;
        }
        return;
    }

    if (!started) {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        started = 1;
    }

    float m = 0.3f * g_sin_wt;
    _pwm_bipolar(m);
}

// ---- OLED display @ 5Hz, 由各拓扑的 g_display_fn 决定显示内容 ----
void Task_Display(void)
{
    if (!g_display_fn) return;

    // I2C 设备就绪检查, 不响应则复位 I2C 外设并重试
    if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 1, 2) != HAL_OK) {
        HAL_I2C_DeInit(&hi2c3);
        HAL_I2C_Init(&hi2c3);
        if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 1, 5) != HAL_OK) {
            static uint8_t fail_cnt = 0;
            if (++fail_cnt >= 10) g_oled_ok = 0;
            return;
        }
        // I2C 恢复, 重新初始化 OLED
        ssd1306_Init();
        ssd1306_Fill(Black);
    }
    g_oled_ok = 1;
    g_display_fn();
}

void UserTasks_Init(void)
{
    Scheduler_Init(20);
    // Control & protect run in ADC ISR @ 10kHz — no scheduler jitter
    Scheduler_AddTask(Task_Button_Scan,         100,    1);
    Scheduler_AddTask(Task_Display,             5,      1);

    // 基线保护: IL1 过流始终开启
    g_protect_mask = PROT_IL1_OC;

    // Select active control mode:
//singlephase
//  g_adc_preproc = adc_preproc_1p;
//  g_display_fn  = Task_Display_1P;
//  g_vofa_fn     = vofa_capture_1p;

//  g_control_isr = Task_Debug_SPWM;       // open-loop debug
//  g_control_isr = Task_Control_OffGrid;  // off-grid inverter
//  g_control_isr = Task_Control_Grid;     // grid-tied inverter
//  g_control_isr = Task_Control_PFC;      // PFC rectifier



//-----------
//dcdc

  g_adc_preproc = adc_preproc_buck;
  g_display_fn  = Task_Display_Buck;
  g_vofa_fn     = vofa_capture_buck;

//  g_control_isr = Task_Debug_Buck;       // Buck 固定占空比调试
  g_control_isr = Task_Control_Buck;     // Buck DCDC
//-----------

    // --- ADC init ---
    Task_ADC_Init();
}

// --- ADC1 Injected ISR
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    uint32_t isr_cyc = DWT->CYCCNT;

    // ISR frequency measurement: count calls, compute kHz every ~0.5s
    static uint32_t isr_cnt = 0, last_cyc = 0;
    isr_cnt++;
    uint32_t dt = isr_cyc - last_cyc;
    if (dt >= SystemCoreClock / 2) {  // ~0.5s
        g_isr_khz = (float)isr_cnt * (float)SystemCoreClock / (float)dt / 1000.0f;
        isr_cnt = 0;
        last_cyc = isr_cyc;
    }

    // 1. Read current (common)
    g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 2050;
    I_line[0] = (float)g_il1 * current_const;

    // 2. Topology-specific ADC preprocessing @ 50kHz
    if (g_adc_preproc)
        g_adc_preproc();

    // 3. Decimate 5:1 → 10kHz for protection, control, VOFA
    static uint8_t decim = 0;
    if (++decim >= 5) {
        decim = 0;

        Task_Protect_Run();
        if (g_control_isr)
            g_control_isr();

        if (g_vofa_fn)
            g_vofa_fn();
    }
}
