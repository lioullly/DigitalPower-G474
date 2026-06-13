#ifndef __TASK_ADC_H
#define __TASK_ADC_H

#include "main.h"

void Task_ADC_Init(void);
extern void (*g_control_isr)(void);

#endif
