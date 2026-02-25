/**
 * @file    GimbalControl.h
 * @brief   云台控制模块头文件
 * @details 云台控制逻辑，包含PID控制、状态管理和锁定检测
 * @version 1.0
 * @date    2026-02-25
 */

#ifndef _GIMBAL_CONTROL_H
#define _GIMBAL_CONTROL_H

#include "stm32f4xx_hal.h"
#include "PID.h"

/**
 * @brief 云台状态枚举
 */
typedef enum {
    GIMBAL_IDLE = 0,     ///< 空闲状态（无目标）
    GIMBAL_TRACKING,     ///< 跟踪状态（正在移动）
    GIMBAL_LOCKED        ///< 锁定状态（到达目标）
} GimbalState;

/**
 * @brief 云台轴枚举
 */
typedef enum {
    GIMBAL_AXIS_H = 0,  ///< 水平轴
    GIMBAL_AXIS_V = 1   ///< 垂直轴
} GimbalAxis;

/**
 * @brief  云台初始化
 * @retval None
 */
void Gimbal_Init(void);

/**
 * @brief  启用云台跟踪
 * @retval None
 */
void Gimbal_Enable(void);

/**
 * @brief  禁用云台跟踪
 * @retval None
 */
void Gimbal_Disable(void);

/**
 * @brief  云台控制任务
 * @retval None
 * @note   在FreeRTOS任务中以50Hz频率调用
 */
void Gimbal_ControlTask(void);

/**
 * @brief  云台自检测试
 * @retval None
 * @note   测试序列：左30° → 右30° → 上15° → 下15°
 */
void Gimbal_SelfTest(void);

/**
 * @brief  获取云台状态
 * @retval 云台状态
 */
GimbalState Gimbal_GetState(void);

/**
 * @brief  设置PID参数
 * @param  axis: 轴选择（水平/垂直）
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void Gimbal_SetPID(GimbalAxis axis, float kp, float ki, float kd);

/**
 * @brief  获取PID参数
 * @param  axis: 轴选择（水平/垂直）
 * @param  kp: 比例系数指针（输出）
 * @param  ki: 积分系数指针（输出）
 * @param  kd: 微分系数指针（输出）
 * @retval None
 */
void Gimbal_GetPID(GimbalAxis axis, float *kp, float *ki, float *kd);

/**
 * @brief  设置调试输出开关
 * @param  enabled: 1=开启, 0=关闭
 * @retval None
 */
void Gimbal_SetDebugOutput(uint8_t enabled);

/**
 * @brief  获取调试输出状态
 * @retval 1=开启, 0=关闭
 */
uint8_t Gimbal_GetDebugOutput(void);

#endif
