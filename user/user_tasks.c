#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
//#include "singlephase/task_display_1p.h"  // OLED not installed
#include "terminal.h"
#include "vofa.h"

// Scheduler tick: TIM6 @ 1MHz (1 tick = 1us)
//   period=100   → 100us → 10kHz (control)
//   period=10000 → 10ms  → 100Hz (button)
//   period=20000 → 20ms  → 50Hz  (VOFA)

void UserTasks_Init(void)
{
    Scheduler_Init(1);

    // --- Pick ONE control task ---
    Scheduler_AddTask(Task_Control_Debug,    100, 1);  // Debug: fixed m=0.8
    //Scheduler_AddTask(Task_Control_OffGrid,  100, 1);  // Off-grid: PI + PR
    //Scheduler_AddTask(Task_Control_GridTied, 100, 1);  // Grid-tied: PLL + PR

    Scheduler_AddTask(Task_Button_Scan, 10000, 1);
    Scheduler_AddTask(Task_VOFA,        20000, 1);
//    Scheduler_AddTask(Task_Terminal,   500,   1);  // text debug
//    Scheduler_AddTask(Task_Display_1P, 200,   1);  // OLED not installed
}
