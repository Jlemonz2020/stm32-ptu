/**
 * @file    Motor.h
 * @brief   电机驱动模块头文件
 * @details 张大头42步闭环步进电机驱动，支持位置模式和速度模式
 * @version 1.0
 * @date    2026-02-25
 */

#ifndef _Motor_H
#define _Motor_H

#include "stm32f4xx_hal.h"
#include "usart.h"

/**
 * @brief  电机初始化
 * @retval None
 * @note   自动使能两个电机（ID=1垂直轴, ID=2水平轴）
 */
void Motor_Init(void);

/**
 * @brief  水平移动
 * @param  angle: 角度（正=右转，负=左转）
 * @retval None
 */
void Motor_MoveHorizontal(float angle);

/**
 * @brief  垂直移动
 * @param  angle: 角度（正=上转，负=下转）
 * @retval None
 */
void Motor_MoveVertical(float angle);

/**
 * @brief  停止所有电机
 * @retval None
 */
void Motor_Stop(void);

/**
 * @brief  设置电机速度（可选）
 * @param  speed_rpm: 速度（RPM）
 * @retval None
 */
void Motor_SetSpeed(uint16_t speed_rpm);

/**
 * @brief  电机失能（可选）
 * @retval None
 */
void Motor_Disable(void);

#endif