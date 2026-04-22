#ifndef __SERIAL_PLOTTER_H
#define __SERIAL_PLOTTER_H

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "drv_uart.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include "drv_math.h"

typedef void (*Plotter_Tx_Func)(uint8_t *data, uint16_t len);

typedef struct {
    Plotter_Tx_Func tx_func;
    //UART编号
    UART_HandleTypeDef *huart;
    //发送缓冲区
    uint8_t *UART_Tx_Data;
    //接收缓冲区
    uint8_t *UART_Rx_Data;
    // 缓冲区位置索引
    uint16_t index;
    // 帧头
    uint8_t Frame_Header;
} SerialPlotter_t;

// 初始化serial_plotter_deveice
void Plotter_Init(SerialPlotter_t *huart, uint8_t *tx_Buffer, uint8_t header, Plotter_Tx_Func tx_func);
// 放入帧头
void Plotter_Begin(SerialPlotter_t *huart);
// 放入帧尾
void Plotter_SendData(SerialPlotter_t *huart);



#endif  //__SERIAL_PLOTTER_H