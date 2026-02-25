/**
 * @file    PID.c
 * @brief   PID控制器实现
 * @details 标准PID算法实现，包含死区处理、积分限幅和输出限幅
 * @version 1.0
 * @date    2026-02-25
 */

#include "PID.h"
#include <math.h>

/**
 * @brief  初始化PID控制器
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    
    pid->integral_max = 100.0f;  // 积分限幅，防止积分饱和
    pid->output_max = 200.0f;    // 输出限幅
    pid->deadzone = 8;           // 死区8像素，避免微小抖动
}

/**
 * @brief  计算PID输出
 * @param  pid: PID控制器指针
 * @param  error: 当前误差值
 * @retval PID输出值
 * @note   包含死区处理、积分限幅和输出限幅
 */
float PID_Calculate(PID_Controller *pid, float error)
{
    // 死区处理
    if (fabsf(error) < pid->deadzone) {
        PID_Reset(pid);
        return 0.0f;
    }
    
    pid->error = error;
    
    // 积分项（带限幅防止积分饱和）
    pid->integral += error;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    // 微分项
    pid->derivative = error - pid->last_error;
    
    // PID输出
    float output = pid->kp * error + 
                   pid->ki * pid->integral + 
                   pid->kd * pid->derivative;
    
    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < -pid->output_max) {
        output = -pid->output_max;
    }
    
    pid->last_error = error;
    
    return output;
}

/**
 * @brief  重置PID控制器状态
 * @param  pid: PID控制器指针
 * @retval None
 * @note   清除误差、积分和微分项
 */
void PID_Reset(PID_Controller *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
}

/**
 * @brief  设置PID参数
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void PID_SetParams(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
