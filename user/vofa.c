#include "vofa.h"
#include "user.h"
#include "usart.h"
#include "timebase_scheduler.h"
#include <string.h>

extern uint8_t g_fault_code;

#define VOFA_FRAME_SIZE  44   // 10 floats + 4-byte tail

static uint8_t vofa_ping[VOFA_FRAME_SIZE];
static uint8_t vofa_pong[VOFA_FRAME_SIZE];
static volatile uint8_t vofa_tx_busy = 0;
static uint8_t vofa_cap_buf = 0;  // 0=ping, 1=pong

// Called from ADC ISR — snapshot + DMA send, all non-blocking
void vofa_capture(void)
{
    float data[10];
    data[0] = I_line[0];        // Ch0: instantaneous current
    data[1] = g_irms;          // Ch1: RMS current
    data[2] = U_line[0];       // Ch2: Uab instantaneous
    data[3] = g_uab_rms;       // Ch3: Uab RMS
    data[4] = U_line[1];       // Ch4: Udc
    data[5] = (float)g_fault_code;  // Ch5: fault
    data[6] = g_dbg_vctrl;     // Ch6: PR output
    data[7] = g_dbg_err;       // Ch7: current error
    data[8] = g_cpu_usage;     // Ch8: CPU usage %
    data[9] = g_isr_us;        // Ch9: ISR time μs

    if (!vofa_tx_busy) {
        uint8_t *dst = vofa_cap_buf ? vofa_pong : vofa_ping;
        memcpy(dst, data, 40);
        dst[40] = 0x00; dst[41] = 0x00;
        dst[42] = 0x80; dst[43] = 0x7F;
        vofa_tx_busy = 1;
        vofa_cap_buf ^= 1;
        HAL_UART_Transmit_DMA(&huart1, dst, VOFA_FRAME_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        vofa_tx_busy = 0;
}
