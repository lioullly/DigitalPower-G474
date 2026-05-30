#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "singlephase/task_protect_1p.h"
#include "singlephase/task_control_1p.h"
#include "vofa.h"

void UserTasks_Init(void)
{
    Scheduler_Init(20);  // HRTIM Master CMP1 @ 50kHz → tick=20us
    Scheduler_AddTask(Task_ADC_Fetch,              50000,  1);  // 50kHz
    Scheduler_AddTask(Task_Control_PI_PR_Loop,         10000,  1);  // debug
    Scheduler_AddTask(Task_Button_Scan,              100,  1);
    Scheduler_AddTask(Task_VOFA_1P,                 5000,  1);
}