#include "task_pwm_1p.h"
#include "hrtim.h"
#include "user.h"

extern float g_duty_a, g_duty_b;

void Task_PWM_1P_Update(void)
{
    if (!Run_Flag) return;

    uint32_t period = 51200;
    uint32_t cmp_a = (uint32_t)(g_duty_a * (float)period);
    uint32_t cmp_b = (uint32_t)(g_duty_b * (float)period);

    if (cmp_a > period) cmp_a = period;
    if (cmp_b > period) cmp_b = period;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}
