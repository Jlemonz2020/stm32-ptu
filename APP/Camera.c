/**
 * @file    Camera.c
 * @brief   视觉数据接收模块实现
 * @details 接收MaixCAM通过USART1发送的目标坐标"X,Y\n"，计算偏差
 * @version 1.0
 * @date    2026-02-25
 */

#include "Camera.h"
#include <string.h>
#include <stdlib.h>

// 调试开关（编译时）
#define DEBUG_CAMERA 1

// 运行时调试控制
static uint8_t camera_debug_enabled = 0;  // 默认关闭调试输出

// MaixCAM图像分辨率和中心点
#define CAMERA_WIDTH    240
#define CAMERA_HEIGHT   240
#define CAMERA_CENTER_X (CAMERA_WIDTH / 2)   // 120
#define CAMERA_CENTER_Y (CAMERA_HEIGHT / 2)  // 120

// 接收缓冲区
static uint8_t camera_rx_buf[32];
static volatile uint16_t camera_rx_index = 0;
static volatile uint8_t camera_data_ready = 0;

// 目标坐标
static int16_t target_x = 0;  // 目标X坐标（0表示无目标）
static int16_t target_y = 0;  // 目标Y坐标（0表示无目标）
static uint8_t target_valid = 0;  // 目标是否有效

/**
 * @brief  初始化摄像头接收模块
 * @retval None
 */
void Camera_Init(void)
{
    memset(camera_rx_buf, 0, sizeof(camera_rx_buf));
    camera_rx_index = 0;
    camera_data_ready = 0;
    target_valid = 0;
    
    // 启动USART1中断接收
    HAL_UART_Receive_IT(&huart1, &camera_rx_buf[camera_rx_index], 1);
}

/**
 * @brief  解析摄像头数据
 * @retval None
 * @note   数据格式: "X,Y\n"，例如"113,114\n"
 */
static void Camera_ParseData(void)
{
    char *comma = strchr((char*)camera_rx_buf, ',');
    if (comma != NULL) {
        *comma = '\0';
        int16_t x = atoi((char*)camera_rx_buf);
        int16_t y = atoi(comma + 1);
        
        // 调试输出：显示接收到的原始数据
        #if DEBUG_CAMERA
        if (camera_debug_enabled) {
            extern void SerialDebug_Printf(const char *format, ...);
            SerialDebug_Printf("[CAM RX] Raw: \"%s,%d\" -> X=%d Y=%d\r\n", 
                              (char*)camera_rx_buf, y, x, y);
        }
        #endif
        
        // 检查是否为有效目标（0,0表示无目标）
        if (x == 0 && y == 0) {
            target_valid = 0;
            camera_data_ready = 0;
            #if DEBUG_CAMERA
            if (camera_debug_enabled) {
                extern void SerialDebug_Printf(const char *format, ...);
                SerialDebug_Printf("[CAM] No target (0,0)\r\n");
            }
            #endif
            return;
        }
        
        // 数据范围检查
        if (x < 0) x = 0;
        if (x > CAMERA_WIDTH) x = CAMERA_WIDTH;
        if (y < 0) y = 0;
        if (y > CAMERA_HEIGHT) y = CAMERA_HEIGHT;
        
        target_x = x;
        target_y = y;
        target_valid = 1;
        camera_data_ready = 1;
        
        #if DEBUG_CAMERA
        if (camera_debug_enabled) {
            extern void SerialDebug_Printf(const char *format, ...);
            SerialDebug_Printf("[CAM] Target valid: (%d,%d)\r\n", x, y);
        }
        #endif
    }
}

/**
 * @brief  UART接收回调函数
 * @retval None
 * @note   在USART1中断中调用，逐字符接收并解析
 */
void Camera_UART_RxCallback(void)
{
    uint8_t received = camera_rx_buf[camera_rx_index];
    
    if (received == '\n' || received == '\r') {
        if (camera_rx_index > 0) {  // 只有当缓冲区有数据时才解析
            camera_rx_buf[camera_rx_index] = '\0';
            Camera_ParseData();
        }
        camera_rx_index = 0;
    } else if ((received >= '0' && received <= '9') || received == ',') {
        // 只接受数字和逗号
        camera_rx_index++;
        if (camera_rx_index >= sizeof(camera_rx_buf) - 1) {
            // 缓冲区溢出，重置
            camera_rx_index = 0;
        }
    } else {
        // 忽略其他字符，不增加索引
    }
    
    // 继续接收下一个字节到当前索引位置
    HAL_UART_Receive_IT(&huart1, &camera_rx_buf[camera_rx_index], 1);
}

/**
 * @brief  尝试获取目标偏差
 * @param  dx: 水平偏差指针（输出）
 * @param  dy: 垂直偏差指针（输出）
 * @retval 1=有新数据, 0=无数据
 * @note   偏差 = 目标位置 - 中心位置(120,120)
 */
int Camera_TryGetDelta(int16_t *dx, int16_t *dy)
{
    if (!camera_data_ready || !target_valid) {
        return 0;
    }
    
    // 计算偏差（目标位置 - 中心位置）
    *dx = target_x - CAMERA_CENTER_X;
    *dy = target_y - CAMERA_CENTER_Y;
    
    camera_data_ready = 0;  // 清除数据就绪标志
    
    return 1;
}

/**
 * @brief  获取目标绝对位置
 * @param  x: X坐标指针（输出）
 * @param  y: Y坐标指针（输出）
 * @retval None
 */
void Camera_GetTargetPosition(int16_t *x, int16_t *y)
{
    *x = target_x;
    *y = target_y;
}

/**
 * @brief  检查目标是否有效
 * @retval 1=有效, 0=无效
 */
uint8_t Camera_IsTargetValid(void)
{
    return target_valid;
}

/**
 * @brief  设置调试输出开关
 * @param  enabled: 1=开启, 0=关闭
 * @retval None
 */
void Camera_SetDebugOutput(uint8_t enabled)
{
    camera_debug_enabled = enabled;
}

/**
 * @brief  获取调试输出状态
 * @retval 1=开启, 0=关闭
 */
uint8_t Camera_GetDebugOutput(void)
{
    return camera_debug_enabled;
}
