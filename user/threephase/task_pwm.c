#include "task_pwm.h"
#include "hrtim.h"
#include "user.h"

extern float g_duty_a, g_duty_b, g_duty_c;

void Task_PWM_Update(void)
{
    if (!Run_Flag)
        return;

    uint32_t period = 51200;
    uint32_t cmp_a = (uint32_t)(g_duty_a * (float)period);
    uint32_t cmp_b = (uint32_t)(g_duty_b * (float)period);
    uint32_t cmp_c = (uint32_t)(g_duty_c * (float)period);

    if (cmp_a > period) cmp_a = period;
    if (cmp_b > period) cmp_b = period;
    if (cmp_c > period) cmp_c = period;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, cmp_c);
}
