#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "singlephase/task_protect_1p.h"
#include "vofa.h"

void UserTasks_Init(void)
{
    Scheduler_Init(1);
//   Scheduler_AddTask(Task_Protect_1P,     10,  1);  // protection
    Scheduler_AddTask(Task_Control_Debug,   10,  1);  // control
    Scheduler_AddTask(Task_Button_Scan,   1000,  1);
//    Scheduler_AddTask(Task_VOFA_1P,      2000,  1);
}
