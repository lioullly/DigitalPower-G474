#include "timebase_scheduler.h"
#include <string.h>

static Task tasks[SCHED_MAX_TASKS];
static uint32_t g_tick_ms = 1;

void Scheduler_Init(uint32_t tick_ms)
{
    g_tick_ms = (tick_ms == 0) ? 1 : tick_ms;
    memset(tasks, 0, sizeof(tasks));
}

int Scheduler_AddTask(TaskFunc func, uint32_t period_ms, uint8_t repeat)
{
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func == 0) {
            tasks[i].func = func;
            tasks[i].period_ms = (period_ms == 0) ? g_tick_ms : period_ms;
            tasks[i].remaining_ms = tasks[i].period_ms;
            tasks[i].repeat = repeat;
            tasks[i].enabled = 1;
            return i;
        }
    }
    return -1;
}

void Scheduler_RemoveTask(int id)
{
    if (id < 0 || id >= SCHED_MAX_TASKS) return;
    tasks[id].func = 0;
    tasks[id].enabled = 0;
}

void Scheduler_Tick(void)
{
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func && tasks[i].enabled) {
            if (tasks[i].remaining_ms <= g_tick_ms) {
                tasks[i].remaining_ms = 0;
            } else {
                tasks[i].remaining_ms -= g_tick_ms;
            }
        }
    }
}

void Scheduler_Dispatch(void)
{
    for (int i = 0; i < SCHED_MAX_TASKS; i++) {
        if (tasks[i].func && tasks[i].enabled && tasks[i].remaining_ms == 0) {
            TaskFunc f = tasks[i].func;
            if (!tasks[i].repeat) {
                tasks[i].enabled = 0;
                tasks[i].func = 0;
            } else {
                tasks[i].remaining_ms = tasks[i].period_ms;
            }
            f();
        }
    }
}
