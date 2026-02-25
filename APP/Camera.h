/**
 * @file    Camera.h
 * @brief   视觉数据接收模块头文件
 * @details 接收MaixCAM发送的目标坐标，计算相对于屏幕中心的偏差
 * @version 1.0
 * @date    2026-02-25
 */

#ifndef _CAMERA_H
#define _CAMERA_H

#include "stm32f4xx_hal.h"
#include "usart.h"

/**
 * @brief  初始化摄像头接收模块
 * @retval None
 * @note   启动USART1中断接收
 */
void Camera_Init(void);

/**
 * @brief  尝试获取目标偏差（相对于屏幕中心）
 * @param  dx: 水平偏差指针（输出）
 * @param  dy: 垂直偏差指针（输出）
 * @retval 1=有新数据, 0=无数据
 */
int Camera_TryGetDelta(int16_t *dx, int16_t *dy);

/**
 * @brief  获取目标绝对位置
 * @param  x: X坐标指针（输出）
 * @param  y: Y坐标指针（输出）
 * @retval None
 */
void Camera_GetTargetPosition(int16_t *x, int16_t *y);

/**
 * @brief  检查目标是否有效
 * @retval 1=有效, 0=无效
 */
uint8_t Camera_IsTargetValid(void);

/**
 * @brief  设置调试输出开关
 * @param  enabled: 1=开启, 0=关闭
 * @retval None
 */
void Camera_SetDebugOutput(uint8_t enabled);

/**
 * @brief  获取调试输出状态
 * @retval 1=开启, 0=关闭
 */
uint8_t Camera_GetDebugOutput(void);

/**
 * @brief  UART接收回调函数
 * @retval None
 * @note   在USART1中断中调用
 */
void Camera_UART_RxCallback(void);

#endif
