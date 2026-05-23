#ifndef __TASK_PROTECT_1P_H
#define __TASK_PROTECT_1P_H

#include "main.h"

#define FAULT_NONE  0
#define FAULT_OV    1
#define FAULT_UV    2
#define FAULT_OC    3
#define FAULT_AC_OV 4

void Task_Protect_1P(void);

extern uint8_t g_fault_code;

#endif
