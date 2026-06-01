/*
 * Task Template — 每个调度器任务的标准结构
 *
 *   void Task_XXX(void)
 *   {
 *       // 1. 边沿检测（必须在 return 之前，否则抓不到下降沿）
 *       static uint8_t prev_active = 0;
 *       uint8_t active = <guard>;          // e.g. Run_Flag, Run_Flag && g_pll_locked
 *       uint8_t rising = (active && !prev_active);
 *       prev_active = active;
 *       if (!active) return;
 *
 *       // 2. 静态状态变量
 *       static float  xxx  = DEFAULT;
 *       static uint16_t dec = 0;
 *
 *       // 3. 一次性初始化（active 升沿触发）
 *       if (rising) {
 *           xxx = DEFAULT;
 *           dec = 0;
 *       }
 *
 *       // 4. 周期工作
 *       if (++dec >= N) { dec = 0;  ... }  // 慢环
 *       // ...                              // 快环（每次调用）
 *   }
 *
 * 规则：每个 task 初始化自包含，不依赖 user_Init 或其他 task。
 */

#include "user_tasks.h"
#include "user.h"
#include "task_adc.h"
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