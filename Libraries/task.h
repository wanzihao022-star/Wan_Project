/**
*@简介：多线程任务的实现
*@作者：铁头山羊
 */

#ifndef __TASK_H_
#define __TASK_H_

#include "main.h"

#define PERIODIC(T) \
static uint32_t nxt = 0; \
if(HAL_GetTick() < nxt) return; \
nxt += (T);

#define PERIODIC_START(NAME, T) \
static uint32_t NAME##_nxt = 0; \
if(HAL_GetTick() >= NAME##_nxt) {\
NAME##_nxt += (T);

#define PERIODIC_END }

#endif /* __TASK_H_ */
