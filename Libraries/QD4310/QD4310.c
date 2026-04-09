/**
*@简介：QD4310电机驱动库，提供了基本的初始化、使能、失能、更新反馈和设置命令的功能。通过CAN总线与电机通信，支持设置目标角度、速度和电流，并且能够读取电机的当前状态。
*@作者：万梓豪
*/
#include "QD4310.h"

#include <math.h>


// 简单的钳位宏，替代 std::clamp

//PM4310_t *dev是要创建的电机对象

// 内部发送函数（私有性质）
static void QD4310_SendCommand(QD4310_t *dev, QD4310_Command_t cmd, int16_t value) {
    uint8_t TxBuffer[3];
    TxBuffer[0] = (uint8_t)cmd;

    // 强制类型转换，注意编译器对齐，如果是ARM可能需要memcpy或位运算
    //*(int16_t *)(TxBuffer + 1) = value;
    TxBuffer[1] = (uint8_t)(value & 0xFF);
    TxBuffer[2] = (uint8_t)((value >> 8) & 0xFF);
    // 或者使用memcpy，确保正确处理对齐问题
    //memcpy(&TxBuffer[1], &value, 2);

    uint32_t txMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x400 + dev->id;
    TxHeader.ExtId = 0x400 + dev->id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 3;

    HAL_CAN_AddTxMessage(dev->hcan, &TxHeader, TxBuffer, &txMailbox);
    // 注意：这里的while循环在嵌入式中要小心，防止卡死
while (HAL_CAN_GetTxMailboxesFreeLevel(dev->hcan) == 0);
}
//初始化函数
void QD4310_Init(QD4310_t *dev, CAN_HandleTypeDef *hcan, uint8_t id) {
    dev->hcan = hcan;
    dev->id = id;
    dev->enabled = 0;
    dev->speed = 0.0f;
    dev->angle = 0.0f;
    dev->current = 0.0f;
}
//使能电机
void QD4310_Enable(QD4310_t *dev) { QD4310_SendCommand(dev, QD4310_CMD_ENABLE, 0); }
//失能电机
void QD4310_Disable(QD4310_t *dev) { QD4310_SendCommand(dev, QD4310_CMD_DISABLE, 0); }

void QD4310_Update(QD4310_t *dev, const uint8_t feedback[8]) {
    dev->enabled = feedback[0] & 0x01;
    // // 使用指针转换获取int16值
    // dev->current = (float)(*(int16_t *)(feedback + 2)) * 10.0f / 32767.0f;
    // dev->speed = (float)(*(int16_t *)(feedback + 4)) * 5000.0f / 32767.0f;
    // dev->angle = (float)(*(uint16_t *)(feedback + 6)) * 2.0f * PM4310_PI / 65535.0f;

    int16_t current_raw = (feedback[3] << 8) | feedback[2];
    int16_t speed_raw   = (feedback[5] << 8) | feedback[4];
    uint16_t angle_raw  = (feedback[7] << 8) | feedback[6];

    dev->current = current_raw * 10.0f / 32767.0f;
    dev->speed   = speed_raw * 1000.0f / 32767.0f;
    dev->angle   = angle_raw * 2.0f * QD4310_PI / 65535.0f;
}

void QD4310_SetAngle(QD4310_t *dev, float angle) {
    // float val = CLAMP(angle, 0.0f, 2.0f * QD4310_PI);
    float val = fmodf(angle, 2.0f * QD4310_PI);
    if (val < 0) val += 2.0f * QD4310_PI;
    uint16_t cmd_val = (uint16_t)(val / (2.0f * QD4310_PI) * 65535.0f);
    QD4310_SendCommand(dev, QD4310_CMD_ANGLE, cmd_val);
}

void QD4310_SetSpeed(QD4310_t *dev, float speed) {
    float val = CLAMP(speed, -1000.0f, 1000.0f);
    int16_t cmd_val = (int16_t)(val / 1000.0f * 32767.0f);
    QD4310_SendCommand(dev, QD4310_CMD_SPEED, cmd_val);
}

void QD4310_SetLowSpeed(QD4310_t *dev, float speed) {
    float val = CLAMP(speed, -1000.0f, 1000.0f);
    int16_t cmd_val = (int16_t)(val / 1000.0f * 32767.0f);
    QD4310_SendCommand(dev, QD4310_CMD_LOW_SPEED, cmd_val);
}

void QD4310_SetCurrent(QD4310_t *dev, float current) {
    float val = CLAMP(current, -10.0f, 10.0f);
    int16_t cmd_val = (int16_t)(val / 10.0f * 32767.0f);
    QD4310_SendCommand(dev, QD4310_CMD_CURRENT, cmd_val);
}

