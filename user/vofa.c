#include "vofa.h"
#include "user.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// Externs from other modules
extern uint8_t g_pll_locked;
extern float   g_freq_est;

static char vofa_buf[256];

void Task_VOFA(void)
{
    // FireWater format: comma-separated floats + \r\n
    // Ch0:Udc  Ch1:IL1  Ch2:IL2  Ch3:IL3  Ch4:Uab  Ch5:Freq  Ch6:DutyA  Ch7:wt
    int len = snprintf(vofa_buf, sizeof(vofa_buf),
        "%.2f,%.3f,%.3f,%.3f,%.2f,%.1f,%.3f,%.3f\r\n",
        (double)U_line[0],     // Ch0: Udc
        (double)I_line[0],     // Ch1: IL1
        (double)I_line[1],     // Ch2: IL2
        (double)I_line[2],     // Ch3: IL3
        (double)U_line[1],     // Ch4: Uab
        (double)g_freq_est,    // Ch5: frequency
        (double)g_duty_a,      // Ch6: duty A
        (double)g_wt           // Ch7: phase angle

    );
    HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, len,100);
}
