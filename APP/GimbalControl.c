/**
 * @file    GimbalControl.c
 * @brief   云台控制模块实现
 * @details 实现双轴PID控制、目标跟踪、锁定检测等功能
 * @version 1.0
 * @date    2026-02-25
 * 
 * @note    控制参数:
 *          - 控制频率: 50Hz (20ms周期)
 *          - PID参数: Kp=150, Ki=0, Kd=0
 *          - 死区: ±8像素
 *          - 锁定判定: 连续10次在死区内
 */

#include "GimbalControl.h"
#include "Camera.h"
#include "Motor.h"
#include "PID.h"
#include "SerialDebug.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// 调试开关（编译时）
#define DEBUG_GIMBAL 1

// 运行时调试控制
static uint8_t debug_output_enabled = 0;  // 默认关闭调试输出

// PID控制器
static PID_Controller pid_h;  // 水平轴
static PID_Controller pid_v;  // 垂直轴

// 云台状态
static GimbalState gimbal_state = GIMBAL_IDLE;
static uint8_t gimbal_enabled = 0;

// 锁定计数器（连续在死区内的次数）
static uint8_t lock_counter = 0;
#define LOCK_THRESHOLD 10  // 连续10次在死区内认为锁定

/**
 * @brief  云台初始化
 * @retval None
 */
void Gimbal_Init(void)
{
    // 初始化PID控制器（提高响应速度）
    PID_Init(&pid_h, 150.0f, 0.00f, 0.0f);  // Kp从100增加到150
    PID_Init(&pid_v, 150.0f, 0.00f, 0.0f);  // Kp从100增加到150

    // 设置死区
    pid_h.deadzone = 8;
    pid_v.deadzone = 8;

    // 初始化相机和电机
    Camera_Init();
    Motor_Init();

    gimbal_state = GIMBAL_IDLE;
    gimbal_enabled = 0;
    lock_counter = 0;
}


/**
 * @brief  启用云台跟踪
 * @retval None
 */
void Gimbal_Enable(void)
{
    gimbal_enabled = 1;
    gimbal_state = GIMBAL_IDLE;
    PID_Reset(&pid_h);
    PID_Reset(&pid_v);
    lock_counter = 0;
}

/**
 * @brief  禁用云台跟踪
 * @retval None
 */
void Gimbal_Disable(void)
{
    gimbal_enabled = 0;
    gimbal_state = GIMBAL_IDLE;
    Motor_Stop();
}

/**
 * @brief  云台控制任务
 * @retval None
 * @note   在FreeRTOS任务中以50Hz频率调用
 */
void Gimbal_ControlTask(void)
{
    if (!gimbal_enabled) return;
    
    int16_t dx = 0, dy = 0;
    int16_t target_x = 0, target_y = 0;
    static uint32_t debug_counter = 0;
    static uint32_t no_data_counter = 0;
    
    // 获取目标位置（用于调试）
    Camera_GetTargetPosition(&target_x, &target_y);
    
    // 获取目标偏差
    if (Camera_TryGetDelta(&dx, &dy))
    {
        gimbal_state = GIMBAL_TRACKING;
        no_data_counter = 0;
        
        // 计算PID输出
        float output_h = PID_Calculate(&pid_h, (float)dx);
        float output_v = PID_Calculate(&pid_v, (float)dy);
        
        // 发送实时数据到上位机
        SerialDebug_SendFeedback(target_x, target_y, dx, dy, output_h, output_v, gimbal_state);
        
        // 每隔10次输出一次调试信息（避免刷屏）
        #if DEBUG_GIMBAL
        if (debug_output_enabled && debug_counter % 10 == 0)
        {
            SerialDebug_Printf("Track: Pos[%d,%d] Delta[%+d,%+d] PID[%.1f,%.1f]\r\n",
                    target_x, target_y, dx, dy, output_h, output_v);
        }
        debug_counter++;
        #endif
        
        // 检查是否在死区内
        if (fabsf(dx) < pid_h.deadzone && fabsf(dy) < pid_v.deadzone)
        {
            lock_counter++;
            if (lock_counter >= LOCK_THRESHOLD)
            {
                gimbal_state = GIMBAL_LOCKED;
                Motor_Stop();
                
                // 发送锁定状态
                SerialDebug_SendFeedback(target_x, target_y, dx, dy, output_h, output_v, gimbal_state);
                
                #if DEBUG_GIMBAL
                if (debug_output_enabled)
                {
                    SerialDebug_Printf(">>> LOCKED at [%d,%d] <<<\r\n", target_x, target_y);
                }
                #endif
                
                lock_counter = 0;  // 重置计数器，避免一直输出
            }
        }
        else
        {
            lock_counter = 0;
            
            // 控制电机移动
            Motor_MoveHorizontal(output_h * 0.01f);  // 转换为角度
            Motor_MoveVertical(output_v * 0.01f);
        }
    }
    else
    {
        // 没有接收到相机数据
        no_data_counter++;
        
        #if DEBUG_GIMBAL
        if (debug_output_enabled)
        {
            if (no_data_counter == 1)  // 只在第一次丢失时输出
            {
                SerialDebug_Printf("Target LOST\r\n");
            }
            if (no_data_counter % 50 == 0)  // 每1秒输出一次
            {
                SerialDebug_Printf("Waiting for camera data... (no data for %d cycles)\r\n", no_data_counter);
            }
        }
        #endif
        
        gimbal_state = GIMBAL_IDLE;
        lock_counter = 0;
    }
}

/**
 * @brief  获取云台状态
 * @retval 云台状态
 */
GimbalState Gimbal_GetState(void)
{
    return gimbal_state;
}

/**
 * @brief  设置PID参数
 * @param  axis: 轴选择（水平/垂直）
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void Gimbal_SetPID(GimbalAxis axis, float kp, float ki, float kd)
{
    if (axis == GIMBAL_AXIS_H)
    {
        PID_SetParams(&pid_h, kp, ki, kd);
    }
    else if (axis == GIMBAL_AXIS_V)
    {
        PID_SetParams(&pid_v, kp, ki, kd);
    }
}

/**
 * @brief  获取PID参数
 * @param  axis: 轴选择（水平/垂直）
 * @param  kp: 比例系数指针（输出）
 * @param  ki: 积分系数指针（输出）
 * @param  kd: 微分系数指针（输出）
 * @retval None
 */
void Gimbal_GetPID(GimbalAxis axis, float *kp, float *ki, float *kd)
{
    if (axis == GIMBAL_AXIS_H)
    {
        *kp = pid_h.kp;
        *ki = pid_h.ki;
        *kd = pid_h.kd;
    }
    else if (axis == GIMBAL_AXIS_V)
    {
        *kp = pid_v.kp;
        *ki = pid_v.ki;
        *kd = pid_v.kd;
    }
}


/**
 * @brief  设置调试输出开关
 * @param  enabled: 1=开启, 0=关闭
 * @retval None
 */
void Gimbal_SetDebugOutput(uint8_t enabled)
{
    debug_output_enabled = enabled;
}

/**
 * @brief  获取调试输出状态
 * @retval 1=开启, 0=关闭
 */
uint8_t Gimbal_GetDebugOutput(void)
{
    return debug_output_enabled;
}
