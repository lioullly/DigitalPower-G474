#ifndef __VOFA_H__
#define __VOFA_H__

// 发送 10 个 float 的 JustFloat 帧 (ping-pong DMA, 非阻塞)
void vofa_send_frame(float data[10]);

// 各拓扑的 VOFA 数据采集函数
void vofa_capture_1p(void);    // 单相
void vofa_capture_buck(void);  // Buck DCDC
void vofa_capture_3p(void);    // 三相

#endif
