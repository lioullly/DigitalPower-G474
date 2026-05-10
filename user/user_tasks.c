#include "user_tasks.h"
#include "user.h"
#include "timebase_scheduler.h"
#include "task_adc.h"
#include "singlephase/task_pwm_1p.h"
#include "singlephase/task_control_1p.h"
#include "singlephase/task_pll_1p.h"
#include "singlephase/task_protect_1p.h"
//#include "singlephase/task_display_1p.h"  // OLED not installed

static int id_adc  = -1;
static int id_pll  = -1;
static int id_pr   = -1;
static int id_pi   = -1;
static int id_pwm  = -1;
static int id_btn  = -1;
static int id_prot = -1;
//static int id_disp = -1;  // OLED not installed

void UserTasks_Init(void)
{
    Scheduler_Init(1);

    id_adc  = Scheduler_AddTask(Task_ADC_Process,       1,  1);
    id_pll  = Scheduler_AddTask(Task_PLL_1P_Process,    1,  1);
    id_pr   = Scheduler_AddTask(Task_PR_CurrentLoop_1P, 1,  1);
    id_pi   = Scheduler_AddTask(Task_PI_VoltageLoop_1P,10,  1);
    id_pwm  = Scheduler_AddTask(Task_PWM_1P_Update,     1,  1);
    id_prot = Scheduler_AddTask(Task_Protect_1P,        1,  1);
    id_btn  = Scheduler_AddTask(Task_Button_Scan,      10,  1);
//    id_disp = Scheduler_AddTask(Task_Display_1P,      200,  1);  // OLED not installed
}