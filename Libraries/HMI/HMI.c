/**
*@简介：陶晶驰串口屏幕驱动，基于HAL库实现，使用空闲中断接收数据，提供设置文本和浮点数的接口
*@作者：万梓豪
*/
#include "HMI.h"
#include <stdio.h>
#include <string.h>

// 淘晶驰屏幕必须的帧尾
const uint8_t HMI_TAIL[3] = {0xFF, 0xFF, 0xFF};
// 接收缓冲区
uint8_t hmi_rx_buf[64];

//开启空闲中断 DMA 接收
//其实这里的代码不是特别的优雅，死绑了串口1，移植的时候记得要把串口改全
void HMI_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hmi_rx_buf, sizeof(hmi_rx_buf));
}
//向串口屏发送文本数据
void HMI_SetText(const char *objName, const char *str) {
    char tx_buf[64];
    int len = snprintf(tx_buf, sizeof(tx_buf), "%s.txt=\"%s\"", objName, str);
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 10);
    HAL_UART_Transmit(&huart1, (uint8_t *)HMI_TAIL, 3, 10);
}
//向串口屏发送浮点变量数据，便于观察数据变化
void HMI_SetFloat(const char *objName, float val) {
    char tx_buf[64];
    int len = snprintf(tx_buf, sizeof(tx_buf), "%s.txt=\"%.2f\"", objName, val);
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 10);
    HAL_UART_Transmit(&huart1, (uint8_t *)HMI_TAIL, 3, 10);
}
