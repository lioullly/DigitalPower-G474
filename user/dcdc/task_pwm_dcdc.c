#include "task_pwm_dcdc.h"
#include "hrtim.h"

#define PWM_PERIOD  51200U

// Buck PWM: Timer A 单半桥, TA1/TA2 互补 + 硬件死区
// duty ∈ [0, 1], 中心对齐, CMP=0 时上管全关
void _pwm_buck(float duty)
{
    if (duty > 0.95f) duty = 0.95f;
    if (duty < 0.00f) duty = 0.00f;

    uint32_t cmp = (uint32_t)(duty * (float)PWM_PERIOD);
    if (cmp > PWM_PERIOD) cmp = PWM_PERIOD;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp);
}
