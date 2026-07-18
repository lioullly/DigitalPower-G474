#ifndef __TASK_PWM_DCDC_H
#define __TASK_PWM_DCDC_H

// Buck PWM: Timer A 单半桥, TA1/TA2 互补 + 硬件死区
// duty ∈ [0, 1], 中心对齐
void _pwm_buck(float duty);

#endif
