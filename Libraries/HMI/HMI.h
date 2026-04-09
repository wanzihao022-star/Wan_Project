
#ifndef QD4310_HMI_H
#define QD4310_HMI_H

#include "usart.h"



// 初始化屏幕接收
void HMI_Init(void);

// 发送文本给屏幕 (如: HMI_SetText("State", "正在复位");)
void HMI_SetText(const char *objName, const char *str);

// 发送浮点数给屏幕 (如: HMI_SetFloat("Yaw_Angle", 45.2);)
void HMI_SetFloat(const char *objName, float val);

#endif //QD4310_HMI_H