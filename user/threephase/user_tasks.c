#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "task_adc.h"
#include "task_pwm.h"
#include "task_control.h"
#include "task_pll.h"

static int id_adc  = -1;
static int id_pll  = -1;
static int id_pr   = -1;
static int id_pi   = -1;
static int id_pwm  = -1;
static int id_btn  = -1;

void UserTasks_Init(void)
{
    Scheduler_Init(1);

    id_adc = Scheduler_AddTask(Task_ADC_Process,     1,  1);
    id_pll = Scheduler_AddTask(Task_PLL_Process,     1,  1);
    id_pr  = Scheduler_AddTask(Task_PR_CurrentLoop,  1,  1);
    id_pi  = Scheduler_AddTask(Task_PI_VoltageLoop, 10,  1);
    id_pwm = Scheduler_AddTask(Task_PWM_Update,     1,  1);
    id_btn = Scheduler_AddTask(Task_Button_Scan,    10,  1);
}
