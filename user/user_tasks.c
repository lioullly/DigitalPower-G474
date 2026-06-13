#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "task_adc.h"
#include "task_protect_1p.h"
#include "vofa.h"
#include "task_pwm_1p.h"
#include "task_control_1p.h"
#include "task_control_pfc.h"
#include "task_control_grid.h"
#include "dac.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

// ---- open-loop debug: fixed m=0.3, 50Hz sine from g_sin_wt ----
void Task_Debug_SPWM(void)
{
    if (!Run_Flag) return;

    static uint8_t started = 0;
    if (!started) {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1,
            HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
            HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
            HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformCounterStart(&hhrtim1,
            HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_F);
        HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(key_relay_GPIO_Port, key_relay_Pin, GPIO_PIN_SET);
        started = 1;
    }

    float m = 0.3f * g_sin_wt;
    _pwm_bipolar(m);
}

// ---- OLED display @ 20Hz (blocks ~1ms, control runs in ISR so no impact) ----
void Task_Display(void)
{
    char buf[24];
    ssd1306_Fill(Black);

    snprintf(buf, sizeof(buf), "Uab %5.0fV %4.1fHz",
             (double)g_uab_rms, (double)g_freq_est);
    ssd1306_SetCursor(0, 2);
    ssd1306_WriteString(buf, Font_11x18, White);

    snprintf(buf, sizeof(buf), "Udc %5.0fV %+4.1fA",
             (double)U_line[1], (double)g_irms);
    ssd1306_SetCursor(0, 22);
    ssd1306_WriteString(buf, Font_11x18, White);

    if (g_fault_code)
        snprintf(buf, sizeof(buf), "FAULT:%1d", g_fault_code);
    else if (!Run_Flag)
        snprintf(buf, sizeof(buf), "STOP");
    else
        snprintf(buf, sizeof(buf), "RUN");
    ssd1306_SetCursor(0, 42);
    ssd1306_WriteString(buf, Font_11x18, White);

    ssd1306_UpdateScreen();
}

void UserTasks_Init(void)
{
    Scheduler_Init(20);
    // Control & protect run in ADC ISR @ 10kHz — no scheduler jitter
    Scheduler_AddTask(Task_Button_Scan,         100,    1);
    Scheduler_AddTask(Task_Display,             20,     1);

    // Select active control mode:
//    g_control_isr = Task_Debug_SPWM;       // open-loop debug
//  g_control_isr = Task_Control_OffGrid;  // off-grid inverter
//  g_control_isr = Task_Control_Grid;     // grid-tied inverter
//  g_control_isr = Task_Control_PFC;      // PFC rectifier
}

// --- ADC1 Injected ISR
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    uint32_t isr_start = DWT->CYCCNT;

    // 1. Read current samples
    g_il1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - 2050;

    // 2. Phase integrator → g_sin_wt → DAC debug output
    float phase = f32_Integral_Calculate(&Sine_Phase_Integrator, 50.0f);
    g_sin_wt = arm_sin_f32(phase * 2.0f * PI);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(g_sin_wt * 2047 + 2048));

    // 3. Convert ADC to SI units (was Task_ADC_Fetch)
    I_line[0] = (float)g_il1 * current_const;
    U_line[0] = (float)(adc2_voltage_buffer[0] - 2036) * U_coefficient;
    U_line[1] = (float)(adc2_voltage_buffer[1] - 0) * U_coefficient;

    // 4. Sliding-window RMS — 1000 samples @ 25kHz
    static float uab_buf[1000], il1_buf[1000];
    static uint16_t idx = 0;
    uab_buf[idx] = U_line[0];
    il1_buf[idx] = I_line[0];
    if (++idx >= 1000) {
        idx = 0;
        arm_rms_f32(uab_buf, 1000, &g_uab_rms);
        arm_rms_f32(il1_buf, 1000, &g_irms);
    }

    // 5. Protection & control — both @ 10kHz, zero scheduling jitter
    Task_Protect_1P_Run();          // check faults, may clear Run_Flag
    if (g_control_isr)
        g_control_isr();

    vofa_capture();  // snapshot telemetry data for VOFA

    // ISR timing — exponential moving average (α=0.05)
    uint32_t isr_elapsed = DWT->CYCCNT - isr_start;
    float isr_us_new = isr_elapsed * (1000000.0f / (float)SystemCoreClock);
    g_isr_us += 0.05f * (isr_us_new - g_isr_us);

    g_adc_data_ready = 1;
}
