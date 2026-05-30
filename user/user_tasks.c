#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "singlephase/task_protect_1p.h"
#include "singlephase/task_control_1p.h"
#include "singlephase/task_control_pfc.h"
#include "singlephase/task_pll_1p.h"
#include "vofa.h"

void UserTasks_Init(void)
{
    Scheduler_Init(20);  // HRTIM Master CMP1 @ 50kHz → tick=20us
    Scheduler_AddTask(Task_ADC_Fetch,              50000,  1);  // 50kHz
    Scheduler_AddTask(Task_PLL_1P_Process,        1000,  1);  // 1kHz PLL
    Scheduler_AddTask(Task_Control_PFC,           10000,  1);
    Scheduler_AddTask(Task_Protect_1P,            10000,  1);  // debug
    Scheduler_AddTask(Task_Button_Scan,              100,  1);
    Scheduler_AddTask(Task_VOFA_1P,                 5000,  1);
}