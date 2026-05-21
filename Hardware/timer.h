/**
 * @file    timer.h
 * @brief   TIM2 定时器模块：20ms 控制节拍
 */

#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

/* ==================== 外部变量声明 ==================== */
extern volatile uint8_t control_flag;   // 20ms 控制节拍标志（在 TIM2 中断中置1）

/* ==================== 函数声明 ==================== */
void TIM2_Init(void);

#endif /* __TIMER_H */