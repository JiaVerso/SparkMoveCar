/**
 * @file drv_uart.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "usart.h"
#include <stdint.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};
Struct_UART_Manage_Object UART7_Manage_Object = {0};
Struct_UART_Manage_Object UART8_Manage_Object = {0};

// UART通信发送缓冲区
uint8_t UART1_Tx_Data[TX_BUF_SIZE];
uint8_t UART2_Tx_Data[TX_BUF_SIZE];
uint8_t UART3_Tx_Data[TX_BUF_SIZE];
uint8_t UART4_Tx_Data[TX_BUF_SIZE];
uint8_t UART5_Tx_Data[TX_BUF_SIZE];
uint8_t UART6_Tx_Data[TX_BUF_SIZE];
uint8_t UART7_Tx_Data[TX_BUF_SIZE];
uint8_t UART8_Tx_Data[TX_BUF_SIZE];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Rx_Buffer 接收缓冲区
 * @param Rx_Buffer_Size 接收缓冲区长度
 * @param Callback_Function 处理回调函数
 */
void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART1_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == USART2)
    {
        UART2_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART2_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART2_Manage_Object.UART_Handler = huart;
        UART2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == USART3)
    {
        UART3_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART3_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART3_Manage_Object.UART_Handler = huart;
        UART3_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == UART4)
    {
        UART4_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART4_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART4_Manage_Object.UART_Handler = huart;
        UART4_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART5_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART5_Manage_Object.UART_Handler = huart;
        UART5_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART6_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART6_Manage_Object.UART_Handler = huart;
        UART6_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART7_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART7_Manage_Object.UART_Handler = huart;
        UART7_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (huart->Instance == UART8)
    {
        UART8_Manage_Object.Rx_Buffer = Rx_Buffer;
        UART8_Manage_Object.Rx_Buffer_Size = Rx_Buffer_Size;
        UART8_Manage_Object.UART_Handler = huart;
        UART8_Manage_Object.Callback_Function = Callback_Function;
    }
    // HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, Rx_Buffer_Size);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, Rx_Buffer_Size);
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}


/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_UART_PeriodElapsedCallback()
{
    // UART串口绘图
    UART_Send_Data(&huart8, UART8_Tx_Data, 1 + 12 * sizeof(float));
}

/**
 * @brief 串口 DMA 循环绕回处理核心算法 (类的方法)
 * @param obj 串口管理对象指针
 * @param Size DMA 当前写到的位置 (HAL库传入)
 */
void UART_Process_Rx_DMA_Circular(Struct_UART_Manage_Object *huart, uint16_t Size)
{
    uint16_t Rx_length; 
    
    // 1. 判断 DMA 指针是否发生了“绕回”
    if (Size >= huart->Rx_buf_pos) 
    {
        // 情况 A：没有绕回，正常顺序接收
        Rx_length = Size - huart->Rx_buf_pos;
        if (Rx_length > 0)
        {
            // 直接调用对象里绑定的回调函数，把干净的一段数据传出去
            if(huart->Callback_Function != NULL) {
                huart->Callback_Function(&huart->Rx_Buffer[huart->Rx_buf_pos], Rx_length);
            }
        }
    } 
    else 
    {
        // 情况 B：发生了绕回！必须分两段提取数据
        uint16_t len1 = huart->Rx_Buffer_Size - huart->Rx_buf_pos;
        if(huart->Callback_Function != NULL) {
            huart->Callback_Function(&huart->Rx_Buffer[huart->Rx_buf_pos], len1);
        }
        
        uint16_t len2 = Size;
        if (len2 > 0 && huart->Callback_Function != NULL)
        {
            huart->Callback_Function(&huart->Rx_Buffer[0], len2);
        }
    }        
    
    // 2. 更新对象的内部读指针
    huart->Rx_buf_pos = Size;

    UART8_Trigger_Tx_DMA();
}


/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    //选择回调函数
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == USART2)
    {
        UART2_Manage_Object.Callback_Function(UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == USART3)
    {
        UART3_Manage_Object.Callback_Function(UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == UART4)
    {
        UART4_Manage_Object.Callback_Function(UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.Callback_Function(UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.Callback_Function(UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Size);
    }
    else if (huart->Instance == UART8)  
    {
        UART8_Manage_Object.Callback_Function(UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Size);
    }
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/