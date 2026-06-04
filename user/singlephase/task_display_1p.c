#include "task_display_1p.h"
#include "task_pll_1p.h"
#include "user.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

static uint8_t inited = 0;
static uint8_t init_failed = 0;
static char buf[24];

void Task_Display_1P(void)
{
    if (!inited && !init_failed)
    {
        if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 2, 10) != HAL_OK) {
            init_failed = 1;
            return;
        }
        ssd1306_Init();
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        inited = 1;
    }
    if (!inited) return;

    ssd1306_Fill(Black);

    snprintf(buf, sizeof(buf), "Uab%6.0fV %5.1fHz", (double)U_line[0], (double)g_freq_est);
    ssd1306_SetCursor(0, 2);
    ssd1306_WriteString(buf, Font_11x18, White);

    snprintf(buf, sizeof(buf), "UDC%6.0fV%5.1fA", (double)U_line[1], (double)I_line[0]);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString(buf, Font_11x18, White);

    if (g_pll_locked && Run_Flag)
        snprintf(buf, sizeof(buf), "GRID:ON  RLY:ON ");
    else if (Run_Flag)
        snprintf(buf, sizeof(buf), "GRID:syncing... ");
    else
        snprintf(buf, sizeof(buf), "STOP           ");
    ssd1306_SetCursor(0, 38);
    ssd1306_WriteString(buf, Font_11x18, White);

    ssd1306_UpdateScreen();
}
