#include "vofa.h"
#include "user.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern float g_freq_est;

static char vofa_buf[256];
static char wave_buf[32];

static void vofa_send(const char *buf, int len)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
}

// JustFloat binary: 5 floats + 4-byte tail (0x00 0x00 0x80 0x7F)
void Task_VOFA_1P(void)
{
    float data[6];
    data[0] = I_line[0];      // Ch0: instantaneous I
    data[1] = g_uab_rms;      // Ch1: Urms
    data[2] = g_irms;         // Ch2: Irms
    data[3] = (float)g_il1;   // Ch3: IL1 offset-corrected
    data[4] = (float)I_mag;   // Ch4: current reference mag
    data[5] = (float)adc2_voltage_buffer[1];  // Ch5: Udc raw
    memcpy(vofa_buf, data, 24);
    vofa_buf[24] = 0x00; vofa_buf[25] = 0x00;
    vofa_buf[26] = 0x80; vofa_buf[27] = 0x7F;
    vofa_send(vofa_buf, 28);
}

// high-speed single-channel: v_ctrl @ 500Hz
void Task_VOFA_Wave(void)
{
    int len = snprintf(wave_buf, sizeof(wave_buf),
        "%.2f\r\n", (double)g_dbg_vctrl);
    vofa_send(wave_buf, len);
}

void Task_VOFA_3P(void)
{
    int len = snprintf(vofa_buf, sizeof(vofa_buf),
        "%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
        (double)U_line[0], (double)I_line[0], (double)I_line[1],
        (double)I_line[2], (double)U_line[1], (double)U_line[2],
        (double)U_line[3], (double)g_freq_est);
    vofa_send(vofa_buf, len);
}
