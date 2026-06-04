#include "vofa.h"
#include "user.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern float g_freq_est;
extern uint8_t g_fault_code;
extern uint8_t g_pll_locked;

static uint8_t vofa_ping[64];    // ping-pong buffer A
static uint8_t vofa_pong[64];    // ping-pong buffer B
static volatile uint8_t vofa_tx_sel = 0;  // 0=next use ping, 1=next use pong
static volatile uint8_t vofa_tx_busy = 0; // DMA transmitting

// fire-and-forget: copies to free buffer, starts DMA. drops frame if busy.
static void vofa_send(const uint8_t *buf, int len)
{
    if (len > 64) return;

    if (!vofa_tx_busy) {
        uint8_t *dst = vofa_tx_sel ? vofa_pong : vofa_ping;
        vofa_tx_sel ^= 1;
        memcpy(dst, buf, len);
        vofa_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, dst, len);
    }
    // else: DMA still running, drop this frame (next one will go through)
}

// Called from USART ISR when TX DMA + UART TC complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        vofa_tx_busy = 0;
    }
}

// JustFloat binary: 10 floats + 4-byte tail (0x00 0x00 0x80 0x7F)
void Task_VOFA_1P(void)
{
    float data[12];
    data[0] = -I_line[0];      // Ch0: instantaneous I
    data[1] = g_uab_rms;      // Ch1: Urms
    data[2] = g_irms;         // Ch2: Irms
    data[3] = (float)g_il1;   // Ch3: IL1 raw
    data[4] = g_dbg_m;        // Ch4: duty_a (via g_dbg_m)
    data[5] = (float)adc2_voltage_buffer[1];  // Ch5: Udc raw
    data[6] = U_line[0];      // Ch6: Uab instantaneous
    data[7] = (float)g_fault_code;            // Ch7: fault code
    data[8] = g_wt;           // Ch8: PLL phase [0,1)
    data[9] = g_freq_est;     // Ch9: PLL frequency
    data[10] = (float)g_pll_locked; // Ch10: grid_mode  
    data[11] = (float)I_mag;  // Ch11: PI output (current mag ref)
    uint8_t frame[52];
    memcpy(frame, data, 48);
    frame[48] = 0x00; frame[49] = 0x00;
    frame[50] = 0x80; frame[51] = 0x7F;
    vofa_send(frame, 52);
}

void Task_VOFA_3P(void)
{
    char text_buf[128];
    int len = snprintf(text_buf, sizeof(text_buf),
        "%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
        (double)U_line[0], (double)I_line[0], (double)I_line[1],
        (double)I_line[2], (double)U_line[1], (double)U_line[2],
        (double)U_line[3], (double)g_freq_est);
    vofa_send((uint8_t*)text_buf, len);
}
