//
// Created by ASUS on 2026/4/5.
//

#ifndef QD4310_LASER_H
#define QD4310_LASER_H
#include "main.h"
//CCR越大，激光笔越亮，CCR越小，激光笔越暗
//ARR = 999，对应 1000 级精度
#define LASER_PWM_MAX     1000
#define LASER_PWM_MIN     0

void Laser_Init(void);
void Laser_SetBrightness(uint16_t percent);  // 0-100%
void Laser_Off(void);
void Laser_On(void);

#endif //QD4310_LASER_H