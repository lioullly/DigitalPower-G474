#ifndef TASK_H
#define TASK_H

#include <stdint.h>

typedef void (*TaskFunc)(void);

typedef struct {
    TaskFunc func;
    uint32_t period_ms;
    uint32_t remaining_ms;
    uint8_t repeat;
    uint8_t enabled;
} Task;

void Task_Init(Task* t, TaskFunc func, uint32_t period_ms, uint8_t repeat);
void Task_Enable(Task* t);
void Task_Disable(Task* t);

#endif
