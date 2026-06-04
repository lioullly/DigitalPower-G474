#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "task_adc.h"
#include "singlephase/task_protect_1p.h"
#include "task_control_1p.h"
#include "singlephase/task_pll_1p.h"
#include "singlephase/task_control_pfc.h"
#include "singlephase/task_display_1p.h"
#include "vofa.h"
#include "task_pwm_1p.h"

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

    g_duty_a = 0.5f + 0.5f * m;
    g_duty_b = 0.5f - 0.5f * m;

    if (g_duty_a > 0.95f) g_duty_a = 0.95f;
    if (g_duty_b > 0.95f) g_duty_b = 0.95f;
    if (g_duty_a < 0.05f) g_duty_a = 0.05f;
    if (g_duty_b < 0.05f) g_duty_b = 0.05f;
    g_duty_c = 0.5f;
    g_dbg_m = g_duty_a;
    Task_PWM_1P_Update();
}

void UserTasks_Init(void)
{
    Scheduler_Init(20);
    Scheduler_AddTask(Task_ADC_Fetch,           10000,  1);
    Scheduler_AddTask(Task_Protect_1P,          10000,  1);
    Scheduler_AddTask(Task_PLL_1P_Process,      1000,   1);
    Scheduler_AddTask(Task_Control_PI_PR_Loop,  10000,  1);
    Scheduler_AddTask(Task_Button_Scan,         100,    1);
    Scheduler_AddTask(Task_VOFA_1P,             5000,   1);
}
