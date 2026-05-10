#ifndef TIMEBASE_SCHEDULER_H
#define TIMEBASE_SCHEDULER_H

#include <stdint.h>
#include "task.h"

#ifndef SCHED_MAX_TASKS
#define SCHED_MAX_TASKS 16
#endif

void Scheduler_Init(uint32_t tick_ms);
int Scheduler_AddTask(TaskFunc func, uint32_t period_ms, uint8_t repeat);
void Scheduler_RemoveTask(int id);
void Scheduler_Tick(void);
void Scheduler_Dispatch(void);

#endif
