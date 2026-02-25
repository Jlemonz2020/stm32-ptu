/**
 * @file    SerialDebug.h
 * @brief   串口调试模块头文件
 * @details 提供串口命令解析、参数调整和调试输出功能
 * @version 1.0
 * @date    2026-02-25
 */

#ifndef _SERIAL_DEBUG_H
#define _SERIAL_DEBUG_H

#include "stm32f4xx_hal.h"

/**
 * @brief  串口调试初始化
 * @retval None
 * @note   启动USART2接收中断，显示欢迎信息
 */
void SerialDebug_Init(void);

/**
 * @brief  处理接收到的命令
 * @retval None
 * @note   在USART2中断中调用
 */
void SerialDebug_ProcessCommand(void);

/**
 * @brief  发送调试信息
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 * @retval None
 */
void SerialDebug_Printf(const char *format, ...);

/**
 * @brief  发送实时数据到上位机
 * @param  target_x: 目标X坐标
 * @param  target_y: 目标Y坐标
 * @param  dx: 水平偏差
 * @param  dy: 垂直偏差
 * @param  pid_h: 水平PID输出
 * @param  pid_v: 垂直PID输出
 * @param  state: 云台状态
 * @retval None
 * @note   格式: DATA,target_x,target_y,dx,dy,pid_h,pid_v,state
 */
void SerialDebug_SendFeedback(int16_t target_x, int16_t target_y, int16_t dx, int16_t dy, 
                               float pid_h, float pid_v, uint8_t state);

/**
 * @brief  获取数据回传状态
 * @retval 1=开启, 0=关闭
 */
uint8_t SerialDebug_IsDataFeedbackEnabled(void);

#endif
