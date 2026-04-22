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
#include "stm32f4xx_hal.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

void Test_Motor_Sweep(void);
void App_Test_SerialPlot_Float(void);

void App_Test_Parse_Command(uint8_t *Buffer, uint16_t Length);
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/