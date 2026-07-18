#include "vofa.h"
#include "user.h"
#include "task_protect.h"
#include "timebase_scheduler.h"
#include "usart.h"
#include <string.h>

#define VOFA_FRAME_SIZE  44   // 10 floats + 4-byte tail

static uint8_t vofa_ping[VOFA_FRAME_SIZE];
static uint8_t vofa_pong[VOFA_FRAME_SIZE];
static volatile uint8_t vofa_tx_busy = 0;
static uint8_t vofa_cap_buf = 0;  // 0=ping, 1=pong

// 发送 JustFloat 帧 (DMA ping-pong, 非阻塞, ISR-safe)
void vofa_send_frame(float data[10])
{
    if (vofa_tx_busy) return;

    uint8_t *dst = vofa_cap_buf ? vofa_pong : vofa_ping;
    memcpy(dst, data, 40);
    dst[40] = 0x00; dst[41] = 0x00;
    dst[42] = 0x80; dst[43] = 0x7F;
    vofa_tx_busy = 1;
    vofa_cap_buf ^= 1;
    HAL_UART_Transmit_DMA(&huart1, dst, VOFA_FRAME_SIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        vofa_tx_busy = 0;
}

// --- 单相 VOFA: 10 通道 ---
void vofa_capture_1p(void)
{
    float data[10];
    data[0] = I_line[0];
    data[1] = g_irms;
    data[2] = U_line[0];
    data[3] = g_uab_rms;
    data[4] = U_line[1];
    data[5] = (float)g_fault_code;
    data[6] = g_dbg_vctrl;
    data[7] = g_dbg_err;
    data[8] = g_cpu_usage;
    data[9] = g_isr_khz;
    vofa_send_frame(data);
}
