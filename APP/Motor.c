/**
 * @file    Motor.c
 * @brief   电机驱动模块实现
 * @details 张大头42步闭环步进电机驱动，支持位置模式和速度模式控制
 * @version 1.0
 * @date    2026-02-25
 * 
 * @note    电机配置:
 *          - Y轴(垂直): ID=1, USART6
 *          - X轴(水平): ID=2, USART3
 *          - 细分: 16细分, 3200脉冲/圈
 *          - 速度: 1200 RPM
 *          - 加速度: 5级
 *          - 校验: 固定0x6B
 */

#include "Motor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ==================== 电机配置 ====================
#define MOTOR_ID_VERTICAL   1    // Y轴（垂直）电机ID
#define MOTOR_ID_HORIZONTAL 2    // X轴（水平）电机ID

// 电机参数
#define MOTOR_STEPS_PER_REV 3200  // 16细分下，3200脉冲=1圈
#define MOTOR_DEGREES_PER_REV 360.0f
#define PULSES_PER_DEGREE (MOTOR_STEPS_PER_REV / MOTOR_DEGREES_PER_REV)  // 约8.89脉冲/度

// 电机速度和加速度（可调整）
#define MOTOR_DEFAULT_SPEED 0x04B0    // 1200 RPM（提高速度，原来600）
#define MOTOR_DEFAULT_ACC   0x05      // 加速度档位5（更快启动，原来10）

// 速度模式参数（备用）
#define MOTOR_SPEED_MODE_RPM  0x0258  // 600 RPM（速度模式，较慢但平滑）
#define MOTOR_SPEED_MODE_ACC  0x0A    // 加速度档位10（平滑加减速）

// 方向定义（已反转以匹配实际安装方向）
#define DIR_CCW 0x00  // 逆时针（反转）
#define DIR_CW  0x01  // 顺时针（反转）

// 命令码
#define CMD_POSITION_CONTROL 0xFD  // 位置模式控制
#define CMD_SPEED_CONTROL    0xF6  // 速度模式控制
#define CMD_STOP             0xFE  // 立即停止
#define CMD_ENABLE           0xF3  // 电机使能控制

// 校验字节（固定）
#define CHECKSUM 0x6B

// 位置模式：相对位置
#define MODE_RELATIVE 0x00

// 多机同步标志：不启用
#define SYNC_DISABLE 0x00

// 控制模式选择
#define USE_SPEED_MODE 0  // 0=使用位置模式（推荐，精确且平滑），1=使用速度模式

// ==================== 内部函数声明 ====================

static void Motor_SendSpeedCommand(uint8_t motor_id, uint8_t direction, uint16_t speed, uint8_t acc);
static void Motor_SendPositionCommand(uint8_t motor_id, int32_t pulses, uint16_t speed, uint8_t acc);
static void Motor_SendStopCommand(uint8_t motor_id);
static void Motor_SendEnableCommand(uint8_t motor_id, uint8_t enable);

// ==================== 内部函数实现 ====================

/**
 * @brief 发送速度控制命令（平滑模式）
 * @param motor_id: 电机ID (1=垂直, 2=水平)
 * @param direction: 方向 (DIR_CW/DIR_CCW)
 * @param speed: 速度 (RPM)
 * @param acc: 加速度档位
 */
static void Motor_SendSpeedCommand(uint8_t motor_id, uint8_t direction, uint16_t speed, uint8_t acc)
{
    uint8_t cmd[8];
    uint8_t index = 0;
    
    // 构建命令帧
    cmd[index++] = motor_id;              // 地址
    cmd[index++] = CMD_SPEED_CONTROL;     // 功能码 0xF6
    cmd[index++] = direction;             // 方向
    cmd[index++] = (speed >> 8) & 0xFF;   // 速度高字节
    cmd[index++] = speed & 0xFF;          // 速度低字节
    cmd[index++] = acc;                   // 加速度档位
    cmd[index++] = SYNC_DISABLE;          // 不启用多机同步
    cmd[index++] = CHECKSUM;              // 校验字节 0x6B
    
    // 根据电机ID选择串口发送
    if (motor_id == MOTOR_ID_VERTICAL)
    {
        HAL_UART_Transmit(&huart6, cmd, index, 100);
    }
    else if (motor_id == MOTOR_ID_HORIZONTAL)
    {
        HAL_UART_Transmit(&huart3, cmd, index, 100);
    }
}

/**
 * @brief 发送位置控制命令
 * @param motor_id: 电机ID (1=垂直, 2=水平)
 * @param pulses: 脉冲数（正=CW，负=CCW）
 * @param speed: 速度 (RPM)
 * @param acc: 加速度档位 (0=不使用曲线)
 */
static void Motor_SendPositionCommand(uint8_t motor_id, int32_t pulses, uint16_t speed, uint8_t acc)
{
    uint8_t cmd[16];
    uint8_t index = 0;
    
    // 确定方向
    uint8_t direction = (pulses >= 0) ? DIR_CW : DIR_CCW;
    uint32_t abs_pulses = (uint32_t)abs(pulses);
    
    // 构建命令帧
    cmd[index++] = motor_id;           // 地址
    cmd[index++] = CMD_POSITION_CONTROL; // 功能码 0xFD
    cmd[index++] = direction;          // 方向
    cmd[index++] = (speed >> 8) & 0xFF; // 速度高字节
    cmd[index++] = speed & 0xFF;        // 速度低字节
    cmd[index++] = acc;                 // 加速度档位
    cmd[index++] = (abs_pulses >> 24) & 0xFF; // 脉冲数[31:24]
    cmd[index++] = (abs_pulses >> 16) & 0xFF; // 脉冲数[23:16]
    cmd[index++] = (abs_pulses >> 8) & 0xFF;  // 脉冲数[15:8]
    cmd[index++] = abs_pulses & 0xFF;         // 脉冲数[7:0]
    cmd[index++] = MODE_RELATIVE;       // 相对位置模式
    cmd[index++] = SYNC_DISABLE;        // 不启用多机同步
    cmd[index++] = CHECKSUM;            // 校验字节 0x6B
    
    // 根据电机ID选择串口发送
    if (motor_id == MOTOR_ID_VERTICAL)
    {
        HAL_UART_Transmit(&huart6, cmd, index, 100);
    }
    else if (motor_id == MOTOR_ID_HORIZONTAL)
    {
        HAL_UART_Transmit(&huart3, cmd, index, 100);
    }
}

/**
 * @brief 发送停止命令
 * @param motor_id: 电机ID
 */
static void Motor_SendStopCommand(uint8_t motor_id)
{
    uint8_t cmd[5];
    
    cmd[0] = motor_id;      // 地址
    cmd[1] = CMD_STOP;      // 功能码 0xFE
    cmd[2] = 0x98;          // 固定参数
    cmd[3] = SYNC_DISABLE;  // 多机同步标志
    cmd[4] = CHECKSUM;      // 校验字节 0x6B
    
    // 根据电机ID选择串口发送
    if (motor_id == MOTOR_ID_VERTICAL)
    {
        HAL_UART_Transmit(&huart6, cmd, 5, 100);
    }
    else if (motor_id == MOTOR_ID_HORIZONTAL)
    {
        HAL_UART_Transmit(&huart3, cmd, 5, 100);
    }
}

/**
 * @brief 发送使能命令
 * @param motor_id: 电机ID
 * @param enable: 1=使能, 0=失能
 */
static void Motor_SendEnableCommand(uint8_t motor_id, uint8_t enable)
{
    uint8_t cmd[6];
    
    cmd[0] = motor_id;      // 地址
    cmd[1] = CMD_ENABLE;    // 功能码 0xF3
    cmd[2] = 0xAB;          // 固定参数
    cmd[3] = enable ? 0x01 : 0x00;  // 使能状态
    cmd[4] = SYNC_DISABLE;  // 多机同步标志
    cmd[5] = CHECKSUM;      // 校验字节 0x6B
    
    // 根据电机ID选择串口发送
    if (motor_id == MOTOR_ID_VERTICAL)
    {
        HAL_UART_Transmit(&huart6, cmd, 6, 100);
    }
    else if (motor_id == MOTOR_ID_HORIZONTAL)
    {
        HAL_UART_Transmit(&huart3, cmd, 6, 100);
    }
}

/**
 * @brief  电机初始化
 * @retval None
 * @note   等待电机上电稳定后使能两个电机
 */
void Motor_Init(void)
{
    // 等待电机上电稳定
    HAL_Delay(100);
    
    // 使能两个电机
    Motor_SendEnableCommand(MOTOR_ID_VERTICAL, 1);
    HAL_Delay(50);
    Motor_SendEnableCommand(MOTOR_ID_HORIZONTAL, 1);
    HAL_Delay(50);
}

/**
 * @brief  水平移动
 * @param  angle: 角度（正=右转，负=左转）
 * @retval None
 * @note   根据USE_SPEED_MODE选择位置模式或速度模式
 */
void Motor_MoveHorizontal(float angle)
{
    if (fabsf(angle) < 0.1f)
    {
        // 角度太小，停止电机
        Motor_SendStopCommand(MOTOR_ID_HORIZONTAL);
        return;
    }
    
#if USE_SPEED_MODE
    // 速度模式：平滑控制
    uint8_t direction = (angle > 0) ? DIR_CW : DIR_CCW;
    
    // 根据角度大小调整速度（角度越大，速度越快）
    uint16_t speed = MOTOR_SPEED_MODE_RPM;
    if (fabsf(angle) > 10.0f)
    {
        speed = MOTOR_SPEED_MODE_RPM * 2;  // 大角度时加速
    }
    
    Motor_SendSpeedCommand(MOTOR_ID_HORIZONTAL, direction, speed, MOTOR_SPEED_MODE_ACC);
    
    // 计算运动时间（修正公式）
    // 时间(ms) = 角度 / (速度RPM * 360度/圈 / 60000ms/分钟)
    float time_ms = fabsf(angle) * 60000.0f / (speed * 360.0f);
    HAL_Delay((uint32_t)time_ms + 100);  // 加100ms余量
    
    // 停止
    Motor_SendStopCommand(MOTOR_ID_HORIZONTAL);
#else
    // 位置模式：快速但可能有冲击
    int32_t pulses = (int32_t)(angle * PULSES_PER_DEGREE);
    Motor_SendPositionCommand(MOTOR_ID_HORIZONTAL, pulses, MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_ACC);
#endif
}

/**
 * @brief  垂直移动
 * @param  angle: 角度（正=上转，负=下转）
 * @retval None
 * @note   根据USE_SPEED_MODE选择位置模式或速度模式
 */
void Motor_MoveVertical(float angle)
{
    if (fabsf(angle) < 0.1f)
    {
        // 角度太小，停止电机
        Motor_SendStopCommand(MOTOR_ID_VERTICAL);
        return;
    }
    
#if USE_SPEED_MODE
    // 速度模式：平滑控制
    uint8_t direction = (angle > 0) ? DIR_CW : DIR_CCW;
    
    // 根据角度大小调整速度
    uint16_t speed = MOTOR_SPEED_MODE_RPM;
    if (fabsf(angle) > 10.0f)
    {
        speed = MOTOR_SPEED_MODE_RPM * 2;  // 大角度时加速
    }
    
    Motor_SendSpeedCommand(MOTOR_ID_VERTICAL, direction, speed, MOTOR_SPEED_MODE_ACC);
    
    // 计算运动时间（修正公式）
    float time_ms = fabsf(angle) * 60000.0f / (speed * 360.0f);
    HAL_Delay((uint32_t)time_ms + 100);  // 加100ms余量
    
    // 停止
    Motor_SendStopCommand(MOTOR_ID_VERTICAL);
#else
    // 位置模式：快速但可能有冲击
    int32_t pulses = (int32_t)(angle * PULSES_PER_DEGREE);
    Motor_SendPositionCommand(MOTOR_ID_VERTICAL, pulses, MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_ACC);
#endif
}

/**
 * @brief  停止所有电机
 * @retval None
 */
void Motor_Stop(void)
{
    // 停止两个电机
    Motor_SendStopCommand(MOTOR_ID_HORIZONTAL);
    Motor_SendStopCommand(MOTOR_ID_VERTICAL);
}

/**
 * @brief  设置电机速度
 * @param  speed_rpm: 速度（RPM）
 * @retval None
 * @note   可选功能，暂未实现
 */
void Motor_SetSpeed(uint16_t speed_rpm)
{
    // 可以在这里保存速度设置，在下次移动时使用
    // 暂时使用默认速度
}

/**
 * @brief  电机失能
 * @retval None
 * @note   可选功能，停止电机并失能
 */
void Motor_Disable(void)
{
    Motor_SendEnableCommand(MOTOR_ID_HORIZONTAL, 0);
    Motor_SendEnableCommand(MOTOR_ID_VERTICAL, 0);
}
