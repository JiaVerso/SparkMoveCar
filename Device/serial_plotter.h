#ifndef __SERIAL_PLOTTER_H
#define __SERIAL_PLOTTER_H

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "drv_uart.h"
#include <math.h>
#include "stm32f4xx_hal.h"

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
// 转换数据类型
void Plotter_AppendFloat(SerialPlotter_t *huart, float val);
void Plotter_AppendInt32(SerialPlotter_t *huart, int32_t val);
void Plotter_AppendInt16(SerialPlotter_t *huart, int16_t val);
void Plotter_AppendUint8(SerialPlotter_t *huart, uint8_t val);

// _Generic关键字实现泛型编程，其实也就是实现C++重载特性。
#define Plotter_Append(obj_ptr, val) _Generic((val), \
    float:     Plotter_AppendFloat, \
    double:    Plotter_AppendFloat, \
    int32_t:   Plotter_AppendInt32, \
    int16_t:   Plotter_AppendInt16, \
    int8_t:    Plotter_AppendInt16, \
    uint32_t:  Plotter_AppendInt32, \
    uint16_t:  Plotter_AppendInt16, \
    uint8_t:   Plotter_AppendUint8, \
    char:      Plotter_AppendUint8  \
)(obj_ptr, val)

#endif  //__SERIAL_PLOTTER_H