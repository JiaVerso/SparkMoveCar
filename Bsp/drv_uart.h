/**
 * @file drv_uart.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_UART_H
#define DRV_UART_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "usart.h"

/* Exported macros -----------------------------------------------------------*/
#define TX_BUF_SIZE
/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UART_Call_Back)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief UART通信处理结构体
 * @param 面向对象
 */
typedef struct {
    UART_HandleTypeDef *UART_Handler;       // 硬件句柄 (HAL库依赖)
    uint8_t *Rx_Buffer;                     // DMA底层物理缓冲区数组    
    uint16_t Rx_Buffer_Size;                // DMA缓冲区的总大小
    uint16_t Rx_buf_pos;                    // Read 指针：记录上一次读取到了哪一位s
    UART_Call_Back Callback_Function;       // 回调业务处理
}Struct_UART_Manage_Object;

/* Exported variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart8;

extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART2_Manage_Object;
extern Struct_UART_Manage_Object UART3_Manage_Object;
extern Struct_UART_Manage_Object UART4_Manage_Object;
extern Struct_UART_Manage_Object UART5_Manage_Object;
extern Struct_UART_Manage_Object UART6_Manage_Object;
extern Struct_UART_Manage_Object UART7_Manage_Object;
extern Struct_UART_Manage_Object UART8_Manage_Object;

extern uint8_t UART1_Tx_Data[];
extern uint8_t UART2_Tx_Data[];
extern uint8_t UART3_Tx_Data[];
extern uint8_t UART4_Tx_Data[];
extern uint8_t UART5_Tx_Data[];
extern uint8_t UART6_Tx_Data[];
extern uint8_t UART7_Tx_Data[];
extern uint8_t UART8_Tx_Data[];

/* Exported function declarations --------------------------------------------*/

void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function);

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

void TIM_UART_PeriodElapsedCallback();

void UART_Process_Rx_DMA_Circular(Struct_UART_Manage_Object *huart, uint16_t Size);

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/