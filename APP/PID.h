/**
 * @file    PID.h
 * @brief   PID控制器头文件
 * @details 实现标准PID控制算法，支持死区、积分限幅和输出限幅
 * @version 1.0
 * @date    2026-02-25
 */

#ifndef _PID_H
#define _PID_H

#include <stdint.h>

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float kp;           ///< 比例系数
    float ki;           ///< 积分系数
    float kd;           ///< 微分系数
    
    float error;        ///< 当前误差
    float last_error;   ///< 上次误差
    float integral;     ///< 积分累积
    float derivative;   ///< 微分项
    
    float integral_max; ///< 积分限幅（防止积分饱和）
    float output_max;   ///< 输出限幅
    
    uint8_t deadzone;   ///< 死区（像素），小于此值不响应
} PID_Controller;

/**
 * @brief  初始化PID控制器
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd);

/**
 * @brief  计算PID输出
 * @param  pid: PID控制器指针
 * @param  error: 当前误差值
 * @retval PID输出值
 */
float PID_Calculate(PID_Controller *pid, float error);

/**
 * @brief  重置PID控制器状态
 * @param  pid: PID控制器指针
 * @retval None
 */
void PID_Reset(PID_Controller *pid);

/**
 * @brief  设置PID参数
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void PID_SetParams(PID_Controller *pid, float kp, float ki, float kd);

#endif
