/**
 * @file    SerialDebug.c
 * @brief   串口调试模块实现
 * @details 实现串口命令解析、参数调整和调试输出功能
 * @version 1.0
 * @date    2026-02-25
 * 
 * @note    支持的命令:
 *          - help: 显示帮助
 *          - status: 显示系统状态
 *          - pid: 设置PID参数
 *          - move: 手动移动电机
 *          - enable/disable: 启用/禁用跟踪
 *          - test: 运行自检
 *          - debug/log/cam: 调试输出控制
 */

#include "SerialDebug.h"
#include "GimbalControl.h"
#include "Motor.h"
#include "Camera.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define RX_BUFFER_SIZE 128
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;
static uint8_t rx_char = 0;  // 当前接收的字符

// 数据回传控制
static uint8_t data_feedback_enabled = 0;
static uint32_t feedback_counter = 0;

/**
 * @brief  串口调试初始化
 * @retval None
 */
void SerialDebug_Init(void)
{
    rx_index = 0;
    
    // 启动UART2接收中断，接收单个字符
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    
    HAL_Delay(100);  // 等待串口稳定
    
    SerialDebug_Printf("\r\n=== Gimbal Serial Debug ===\r\n");
    SerialDebug_Printf("Commands:\r\n");
    SerialDebug_Printf("  help          - Show this help\r\n");
    SerialDebug_Printf("  status        - Show system status\r\n");
    SerialDebug_Printf("  pid h <kp> <ki> <kd> - Set horizontal PID\r\n");
    SerialDebug_Printf("  pid v <kp> <ki> <kd> - Set vertical PID\r\n");
    SerialDebug_Printf("  move h <angle>       - Move horizontal (degrees)\r\n");
    SerialDebug_Printf("  move v <angle>       - Move vertical (degrees)\r\n");
    SerialDebug_Printf("  stop          - Stop motors\r\n");
    SerialDebug_Printf("  enable        - Enable gimbal control\r\n");
    SerialDebug_Printf("  disable       - Disable gimbal control\r\n");
    SerialDebug_Printf("  test          - Run self test\r\n");
    SerialDebug_Printf("  debug on/off  - Enable/disable data feedback\r\n");
    SerialDebug_Printf("  log on/off    - Enable/disable debug output\r\n");
    SerialDebug_Printf("  cam on/off    - Enable/disable camera debug\r\n");
    SerialDebug_Printf("===========================\r\n\n");
}

/**
 * @brief  发送调试信息
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 * @retval None
 */
void SerialDebug_Printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
    }
}

/**
 * @brief  处理命令字符串
 * @param  cmd: 命令字符串
 * @retval None
 */
static void ProcessCommand(char *cmd)
{
    // 去除换行符
    char *newline = strchr(cmd, '\r');
    if (newline) *newline = '\0';
    newline = strchr(cmd, '\n');
    if (newline) *newline = '\0';
    
    if (strlen(cmd) == 0) return;
    
    SerialDebug_Printf("> %s\r\n", cmd);
    
    // help命令
    if (strcmp(cmd, "help") == 0)
    {
        SerialDebug_Printf("Commands:\r\n");
        SerialDebug_Printf("  help          - Show this help\r\n");
        SerialDebug_Printf("  status        - Show system status\r\n");
        SerialDebug_Printf("  pid h <kp> <ki> <kd> - Set horizontal PID\r\n");
        SerialDebug_Printf("  pid v <kp> <ki> <kd> - Set vertical PID\r\n");
        SerialDebug_Printf("  deadzone <value>     - Set deadzone (pixels)\r\n");
        SerialDebug_Printf("  move h <angle>       - Move horizontal (degrees)\r\n");
        SerialDebug_Printf("  move v <angle>       - Move vertical (degrees)\r\n");
        SerialDebug_Printf("  stop          - Stop motors\r\n");
        SerialDebug_Printf("  enable        - Enable gimbal control\r\n");
        SerialDebug_Printf("  disable       - Disable gimbal control\r\n");
        SerialDebug_Printf("  test          - Run self test\r\n");
        SerialDebug_Printf("  debug on/off  - Enable/disable data feedback\r\n");
        SerialDebug_Printf("  log on/off    - Enable/disable debug output\r\n");
        SerialDebug_Printf("  cam on/off    - Enable/disable camera debug\r\n");
    }
    // status命令
    else if (strcmp(cmd, "status") == 0)
    {
        float kp_h, ki_h, kd_h, kp_v, ki_v, kd_v;
        Gimbal_GetPID(GIMBAL_AXIS_H, &kp_h, &ki_h, &kd_h);
        Gimbal_GetPID(GIMBAL_AXIS_V, &kp_v, &ki_v, &kd_v);
        
        GimbalState state = Gimbal_GetState();
        const char *state_str[] = {"IDLE", "TRACKING", "LOCKED"};
        
        SerialDebug_Printf("=== System Status ===\r\n");
        SerialDebug_Printf("State: %s\r\n", state_str[state]);
        SerialDebug_Printf("PID_H: Kp=%.2f Ki=%.3f Kd=%.2f\r\n", kp_h, ki_h, kd_h);
        SerialDebug_Printf("PID_V: Kp=%.2f Ki=%.3f Kd=%.2f\r\n", kp_v, ki_v, kd_v);
        SerialDebug_Printf("====================\r\n");
    }
    // pid命令
    else if (strncmp(cmd, "pid ", 4) == 0)
    {
        char axis;
        float kp, ki, kd;
        if (sscanf(cmd + 4, "%c %f %f %f", &axis, &kp, &ki, &kd) == 4)
        {
            if (axis == 'h' || axis == 'H')
            {
                Gimbal_SetPID(GIMBAL_AXIS_H, kp, ki, kd);
                SerialDebug_Printf("Horizontal PID set: Kp=%.2f Ki=%.3f Kd=%.2f\r\n", kp, ki, kd);
            }
            else if (axis == 'v' || axis == 'V')
            {
                Gimbal_SetPID(GIMBAL_AXIS_V, kp, ki, kd);
                SerialDebug_Printf("Vertical PID set: Kp=%.2f Ki=%.3f Kd=%.2f\r\n", kp, ki, kd);
            }
            else
            {
                SerialDebug_Printf("Error: Invalid axis (use 'h' or 'v')\r\n");
            }
        }
        else
        {
            SerialDebug_Printf("Error: Usage: pid <h|v> <kp> <ki> <kd>\r\n");
        }
    }
    // move命令
    else if (strncmp(cmd, "move ", 5) == 0)
    {
        char axis;
        float angle;
        if (sscanf(cmd + 5, "%c %f", &axis, &angle) == 2)
        {
            if (axis == 'h' || axis == 'H')
            {
                Motor_MoveHorizontal(angle);
                SerialDebug_Printf("Moving horizontal: %.2f degrees\r\n", angle);
            }
            else if (axis == 'v' || axis == 'V')
            {
                Motor_MoveVertical(angle);
                SerialDebug_Printf("Moving vertical: %.2f degrees\r\n", angle);
            }
            else
            {
                SerialDebug_Printf("Error: Invalid axis (use 'h' or 'v')\r\n");
            }
        }
        else
        {
            SerialDebug_Printf("Error: Usage: move <h|v> <angle>\r\n");
        }
    }
    // stop命令
    else if (strcmp(cmd, "stop") == 0)
    {
        Motor_Stop();
        SerialDebug_Printf("Motors stopped\r\n");
    }
    // enable命令
    else if (strcmp(cmd, "enable") == 0)
    {
        Gimbal_Enable();
        SerialDebug_Printf("Gimbal control enabled\r\n");
    }
    // disable命令
    else if (strcmp(cmd, "disable") == 0)
    {
        Gimbal_Disable();
        SerialDebug_Printf("Gimbal control disabled\r\n");
    }
    // test命令
    else if (strcmp(cmd, "test") == 0)
    {
        SerialDebug_Printf("Running self test...\r\n");
        Gimbal_SelfTest();
        SerialDebug_Printf("Self test completed\r\n");
    }
    // debug命令 - 开启/关闭实时数据回传
    else if (strcmp(cmd, "debug on") == 0)
    {
        data_feedback_enabled = 1;
        SerialDebug_Printf("Data feedback enabled\r\n");
    }
    else if (strcmp(cmd, "debug off") == 0)
    {
        data_feedback_enabled = 0;
        SerialDebug_Printf("Data feedback disabled\r\n");
    }
    // log命令 - 开启/关闭调试输出
    else if (strcmp(cmd, "log on") == 0)
    {
        Gimbal_SetDebugOutput(1);
        SerialDebug_Printf("Debug output enabled\r\n");
    }
    else if (strcmp(cmd, "log off") == 0)
    {
        Gimbal_SetDebugOutput(0);
        SerialDebug_Printf("Debug output disabled\r\n");
    }
    // cam命令 - 开启/关闭相机调试输出
    else if (strcmp(cmd, "cam on") == 0)
    {
        Camera_SetDebugOutput(1);
        SerialDebug_Printf("Camera debug output enabled\r\n");
    }
    else if (strcmp(cmd, "cam off") == 0)
    {
        Camera_SetDebugOutput(0);
        SerialDebug_Printf("Camera debug output disabled\r\n");
    }
    else
    {
        SerialDebug_Printf("Unknown command. Type 'help' for available commands.\r\n");
    }
}

/**
 * @brief  处理接收到的命令
 * @retval None
 * @note   在USART2中断中调用
 */
void SerialDebug_ProcessCommand(void)
{
    // 这个函数在UART接收完成中断中调用
    // 将接收到的字符存入缓冲区
    if (rx_char == '\n' || rx_char == '\r')
    {
        if (rx_index > 0)  // 只有当缓冲区有内容时才处理
        {
            rx_buffer[rx_index] = '\0';
            ProcessCommand((char*)rx_buffer);
            rx_index = 0;
        }
    }
    else if (rx_char >= 32 && rx_char <= 126)  // 只接受可打印字符
    {
        if (rx_index < RX_BUFFER_SIZE - 1)
        {
            rx_buffer[rx_index++] = rx_char;
        }
        else
        {
            rx_index = 0;
            SerialDebug_Printf("\r\nError: Command too long\r\n");
        }
    }
    
    // 继续接收下一个字符
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
}

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
                               float pid_h, float pid_v, uint8_t state)
{
    if (!data_feedback_enabled) return;
    
    // 每10次发送一次，避免刷屏
    feedback_counter++;
    if (feedback_counter % 10 != 0) return;
    
    // 格式: DATA,target_x,target_y,dx,dy,pid_h,pid_v,state\n
    // 方便上位机解析
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "DATA,%d,%d,%d,%d,%.1f,%.1f,%d\r\n",
             target_x, target_y, dx, dy, pid_h, pid_v, state);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}

/**
 * @brief  获取数据回传状态
 * @retval 1=开启, 0=关闭
 */
uint8_t SerialDebug_IsDataFeedbackEnabled(void)
{
    return data_feedback_enabled;
}
