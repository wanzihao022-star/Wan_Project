//
// Created by ASUS on 2026/3/28.
//

/****
 * @简介：C语言实现CAN总线通信控制Qrive4310无刷伺服电机
 * by: 2026-03-16 万梓豪
 */

#ifndef __QD4310_H__
#define __QD4310_H__

#include <stdint.h>
#include "can.h"
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define QD4310_PI 3.1415926535f
#define RAD 57.2958f           //*RAD表示把弧度转化为角度
#define DU 0.017453f           //*DU表示把角度转化为弧度



//电机参数结构体
typedef struct
{
    CAN_HandleTypeDef *hcan; //CAN总线句柄
    uint8_t id;    //电机ID
    //状态参数
    uint8_t enabled;
    float speed;
    float angle;
    float current; //电流-10A-10A
}QD4310_t;

//电机命令枚举结构体
typedef enum
{
    QD4310_Cmd_NOP = 0x00,     //用于获取反馈报文
    QD4310_CMD_ENABLE = 0x01,   //使能电机
    QD4310_CMD_DISABLE = 0x02,  //禁用电机
    QD4310_CMD_CURRENT = 0x03, //设置电流
    QD4310_CMD_SPEED = 0x04,   //设置速度
    QD4310_CMD_ANGLE = 0x05, //设置角度
    QD4310_CMD_LOW_SPEED = 0x06 //设置电流限制低速模式
} QD4310_Command_t;

//函数声明

void QD4310_Init(QD4310_t *dev, CAN_HandleTypeDef *hcan, uint8_t id);
void QD4310_Enable(QD4310_t *dev);
void QD4310_Disable(QD4310_t *dev);

/**
 * @param dev 电机对象
 * @param 反馈电机状态
 */
void QD4310_Update(QD4310_t *dev, const uint8_t feedback[8]);
/**
* @brief 设置电机角度
* @param _angle 设置的角度,[0,2pi] 单位rad
*/
void QD4310_SetAngle(QD4310_t *dev, float angle);

/**
* @brief 设置电机转速
* @param _speed 设置的转速,[-1000,1000] 单位rpm
*/
void QD4310_SetSpeed(QD4310_t *dev, float speed);

/**
 * @brief 设置电机转速
 * @param _speed 设置的转速,[-100,100] 单位rpm
 */
void QD4310_SetLowSpeed(QD4310_t *dev, float speed);

/**
 * @brief 设置电机电流
 * @param _current 设置的转速,[-10,10] 单位A
 */
void QD4310_SetCurrent(QD4310_t *dev, float current);

#endif /* __QD4310_H__ */