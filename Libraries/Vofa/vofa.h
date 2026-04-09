//
// Created by ASUS on 2026/3/28.
//

#ifndef QD4310_VOFA_H
#define QD4310_VOFA_H


#include "main.h" // 包含HAL库

// 定义在VOFA+里同时观察几根线 (几条波形)
// 这里设为3，分别对应：角度、速度、电流
#define VOFA_CH_COUNT 3

// 函数声明
void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_Send_JustFloat(float ch0, float ch1, float ch2);


#endif //QD4310_VOFA_H