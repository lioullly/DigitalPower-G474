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

