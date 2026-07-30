#ifndef __TASK_PWM_1P_H
#define __TASK_PWM_1P_H

#include "main.h"

void Task_PWM_1P_Update(void);
void Task_PWM_1P_UniUpdate(float m, float v_alpha);
void _pwm_bipolar(float m);

#endif
