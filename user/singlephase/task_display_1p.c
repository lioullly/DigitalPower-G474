#include "task_display_1p.h"
#include "user.h"
#include "task_protect.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

void Task_Display_1P(void)
{
    char buf[24];
    ssd1306_Fill(Black);

    snprintf(buf, sizeof(buf), "Uac%5.0fV %4.1fHz",
             (double)g_uab_rms, (double)g_freq_est);
    ssd1306_SetCursor(0, 2);
    ssd1306_WriteString(buf, Font_7x10, White);

    snprintf(buf, sizeof(buf), "Udc%5.0fV %+4.1fA",
             (double)U_line[1], (double)g_irms);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString(buf, Font_7x10, White);

    if (g_fault_code)
        snprintf(buf, sizeof(buf), "FAULT:%d", g_fault_code);
    else if (!Run_Flag)
        snprintf(buf, sizeof(buf), "STOP  %s", g_task_name);
    else
        snprintf(buf, sizeof(buf), "RUN   %s", g_task_name);
    ssd1306_SetCursor(0, 28);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}
