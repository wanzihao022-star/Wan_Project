/**
*@简介：VOFA+ 协议的实现文件，提供了初始化和发送数据的函数。
*但是之后和相机建立联系后，没有多余的串口使用了，因此用不上喽
*@作者：万梓豪
*/
#include "vofa.h"

// 记录绑定的串口句柄 (比如 huart1)
static UART_HandleTypeDef *vofa_uart = NULL;

// JustFloat 协议的数据包结构 (必须1字节对齐)
#pragma pack(push, 1)
struct JustFloatField {
    float fdata[VOFA_CH_COUNT];
    uint8_t tail[4]; // VOFA+ 要求的专属帧尾
};
#pragma pack(pop)

// 实例化数据包，并初始化帧尾
static struct JustFloatField vofa_tx = {
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

// 初始化：绑定串口
void VOFA_Init(UART_HandleTypeDef *huart) {
    vofa_uart = huart;
}

// 发送波形数据
void VOFA_Send_JustFloat(float ch0, float ch1, float ch2) {
    if (vofa_uart == NULL) return; // 防错

    // 装载数据
    vofa_tx.fdata[0] = ch0;
    vofa_tx.fdata[1] = ch1;
    vofa_tx.fdata[2] = ch2;

    // 通过串口发送
    // 参数：串口句柄、数据包首地址、数据包总长度、超时时间
    HAL_UART_Transmit(vofa_uart, (uint8_t *)&vofa_tx, sizeof(vofa_tx), 10);

    // 提示：如果发现加上这行代码后电机转得有卡顿感，
    // 说明 115200 的波特率发数据太慢阻塞了CPU。
    // 解决方案1：把 CubeMX 里的串口波特率提高到 460800 或 921600。
    // 解决方案2：开启串口的 TX DMA，并将上一行替换为：
    // HAL_UART_Transmit_DMA(vofa_uart, (uint8_t *)&vofa_tx, sizeof(vofa_tx));
}