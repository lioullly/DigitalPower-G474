#include "task_display_dcdc.h"
#include "user.h"
#include "task_protect.h"
#include "hardware_def_dcdc.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

void Task_Display_Buck(void)
{
    char buf[24];
    ssd1306_Fill(Black);

    // 第一行: Vin + Vout
    snprintf(buf, sizeof(buf), "Vi%4.0f Vo%4.0fV",
             (double)U_line[2], (double)U_line[3]);
    ssd1306_SetCursor(0, 2);
    ssd1306_WriteString(buf, Font_7x10, White);

    // 第二行: IL + duty
    snprintf(buf, sizeof(buf), "IL%+5.1fA D%3.0f%%",
             (double)I_line[0], (double)(g_dbg_vctrl * 100.0f));
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString(buf, Font_7x10, White);

    // 第三行: 状态
    if (g_fault_code)
        snprintf(buf, sizeof(buf), "FAULT:%d", g_fault_code);
    else if (!Run_Flag)
        snprintf(buf, sizeof(buf), "STOP");
    else
        snprintf(buf, sizeof(buf), "RUN Iref%4.1fA", (double)g_dbg_err);
    ssd1306_SetCursor(0, 28);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
}
