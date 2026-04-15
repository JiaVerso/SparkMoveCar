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
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

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