#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "task_adc.h"
#include "task_protect.h"
#include "vofa.h"
#include "task_control_1p.h"
#include "task_control_pfc.h"
#include "task_control_grid.h"
#include "task_display_1p.h"

void UserTasks_Init(void)
{
    Scheduler_Init(50);  // tick = 50μs (20kHz PWM, up-counting CMP1 match)
    // Control & protect run in ADC ISR @ 10kHz — no scheduler jitter
    Scheduler_AddTask(Task_Button_Scan,         100,    1);
    Scheduler_AddTask(Task_Display,             5,      1);


    // Select active control mode:
//singlephase
  g_adc_preproc = adc_preproc_1p;g_display_fn  = Task_Display_1P;g_vofa_fn     = vofa_capture_1p;

 //g_control_isr = Task_Debug_SPWM;       g_task_name = "Debug";
  //g_control_isr = Task_Control_OffGrid;  g_task_name = "OffGrid";
//  g_control_isr = Task_Control_Grid;     g_task_name = "Grid";
  g_control_isr = Task_Control_PFC;        g_task_name = "PFC";
}

