/**
 * @file app_test.c
 * @author JiaVerso
 * @brief 应用测试层
 * @version 0.1
 * @date 2026-04-015
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

 /* USER CODE END Header */
#include "app_test.h"
#include "drv_can.h"
#include "drv_uart.h" 
#include "pidcontroller.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "main.h"
/* USER CODE END Includes */

extern PID_t pid_speed;

/**
 * @brief  M3508 电机平滑正反转测试函数 (ID=2)
 * @note   这是一个阻塞型测试函数，仅供开机自检或裸机调试时使用！
 * 调用此函数期间，单片机会停留在此处数秒。
 */
void Test_Motor_Sweep(void)
{
    int16_t current = 0;
    
    // 正转加速
    while (current < 4000) {
        current += 10;
        CAN1_0x200_Tx_Data[2] = current >> 8;
        CAN1_0x200_Tx_Data[3] = current;
        CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
        HAL_Delay(10);
    }
    
    // 反转减速到反向加速
    while (current > -4000) {
        current -= 10;
        CAN1_0x200_Tx_Data[2] = current >> 8;
        CAN1_0x200_Tx_Data[3] = current;
        CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
        HAL_Delay(10);
    }
    
    // 归零保护（测试结束一定要让电机停下来！）
    CAN1_0x200_Tx_Data[2] = 0;
    CAN1_0x200_Tx_Data[3] = 0;
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
}

/**
 * @brief  串口多路浮点数发送测试 (用于 SerialPlot 波形查看)
 * @note   建议在 main.c 的 while(1) 中每隔 10ms 调用一次此函数
 * @note   tmp_data 4字节 缓存数组uint8_t ，需要将float的内存地址重解释。((char *)(&tmp_data) + i)
 * @note   ((char *)(&tmp_data) + i) 告诉编译器这是一连串单字节的字符指针，接受端按float接收在强转为float（*(float *)rx_buffer）
 */
void App_Test_SerialPlot_Float(void)
{
    static uint16_t flag = 0;
    
    static uint8_t test_tx_buf[9]; 
    test_tx_buf[0] = 0xAA;
    
    // 业务逻辑 1：生成一个 0 到 6.25 的抛物线波形
    if (flag == 2500) {
        flag = 0;
    }
    
    float tmp_data = ((float)flag / 1000.0f) * ((float)flag / 1000.0f);
    for (uint8_t i = 0; i < 4; i++) {
        test_tx_buf[i + 1] = *((char *)(&tmp_data) + i); 
    }

    // 业务逻辑 2：读取按键/LED 状态波形
    // 读出来的 0 或 1 存入 float，这在 SerialPlot 里会画出完美的方波！
    float led_status = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1);
    for (uint8_t i = 0; i < 4; i++) {
        // 数据放在 [5] 到 [8]
        test_tx_buf[i + 5] = *((char *)(&led_status) + i);
    }

    // 调用底层驱动发送这 9 个字节
    UART_Send_Data(&huart8, test_tx_buf, 9);
    
    // 状态更新
    flag++;
    
}

/**
 * @brief  解析serialplot发送过来的pid参数
 * @note   这是一个阻塞型测试函数，仅供开机自检或裸机调试时使用！
 * @param  "PID=P,I,D"  
 */
void App_Test_Parse_Command(uint8_t *Buffer, uint16_t Length)
{
    char *str = (char *)Buffer;

    // int strncmp(const char *str1, const char *str2, size_t n) -- String n Compare
    // float strtof(const char *nptr, char **endptr) -- String to Float
    if (strncmp(str, "PID=", 4) == 0) 
    {
        char *pEnd;
        float p_val = strtof(str + 4, &pEnd);
        if (*pEnd == ',') 
        {
            float i_val = strtof(pEnd + 1, &pEnd);
            if (*pEnd == ',') 
            {
                float d_val = strtof(pEnd + 1, &pEnd);
                if(*pEnd == ',')
                {
                    float f_val = strtof(pEnd + 1, NULL);
                    // 赋值
                    pid_speed.Kp = p_val;
                    pid_speed.Ki = i_val;
                    pid_speed.Kd = d_val;
                    pid_speed.Kf = f_val;
                }
               
            }

        }
    }
}