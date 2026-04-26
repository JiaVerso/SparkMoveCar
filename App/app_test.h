/**
 * @file app_test.h
 * @author JiaVerso
 * @brief 应用测试层
 * @version 0.1
 * @date 2026-04-015
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef __APP_TEST_H
#define __APP_TEST_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* ---------------------------------------------------------------------------*/
#include "drv_math.h"
#include "fifo.h"
#include "stm32f4xx_hal.h"
/* Exported macros -----------------------------------------------------------*/

#define KEY_PIN             GPIO_PIN_10
#define KEY_PORT            GPIOH
#define LONG_PRESS_TIME_MS  3000  // 定义长按阈值为 3000 毫秒

/* Exported types ------------------------------------------------------------*/

void Test_Motor_Sweep(void);
void App_Test_SerialPlot_Float(void);

void App_Test_Parse_Command(uint8_t *Buffer, uint16_t Length);
void App_Test_Trigger_UART_DMA(UART_HandleTypeDef *huart, fifo_s_t *fifo, uint8_t *dma_buf, uint16_t max_buf_size);
//  void App_Test_Motor_FeedBack(void);
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/