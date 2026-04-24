/**
 * @file serial_plotter.c
 * @author JiaVerso
 * @brief Deveice设备层
 * @version 0.1
 * @date 2026-04-15
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#include "serial_plotter.h"
#include <stddef.h>

// 用于拆分 float 的联合体工具
typedef union {
    float float_val;
    uint8_t bytes[4];
} FloatConverter_t;


// 初始化
void Plotter_Init(SerialPlotter_t *huart, uint8_t *tx_Buffer, uint8_t header, Plotter_Tx_Func tx_func) {
    huart->Frame_Header = header;
    huart->tx_func = tx_func;
    huart->UART_Tx_Data = tx_Buffer;
    huart->index = 0;
}

void Plotter_Begin(SerialPlotter_t *huart) {
    if (huart == NULL) return;
    // 写入帧头
    huart->UART_Tx_Data[huart->index++] = huart->Frame_Header;
}

void Plotter_SendData(SerialPlotter_t *huart) {
     // 安全校验
    if (huart == NULL) {
        return; 
    }
    // 调用回调函数将 tx_buffer 发送出去
    huart->tx_func(huart->UART_Tx_Data, huart->index);
    huart->index = 0;
}

/**
************************************************************************
* @brief:       Plotter_AppendFloat: 追加单精度浮点数函数
* @param[in]:   huart:      指向串口绘图器结构体的句柄指针
* @param[in]:   val:        待发送的单精度浮点数 (float)
* @retval:      无 (void)
* @details:     利用共用体 (FloatConverter_t) 的内存共享特性，将32位浮点数
* 无损拆分为4个独立的字节，并依次追加到 UART 发送缓存区中，
* 同时自动递增缓存区数据索引。
************************************************************************
**/
void Plotter_AppendFloat(SerialPlotter_t *huart, float val) {
    FloatConverter_t fu;
    fu.float_val = val;
    huart->UART_Tx_Data[huart->index++] = fu.bytes[0];
    huart->UART_Tx_Data[huart->index++] = fu.bytes[1];
    huart->UART_Tx_Data[huart->index++] = fu.bytes[2];
    huart->UART_Tx_Data[huart->index++] = fu.bytes[3];
}

void Plotter_AppendInt16(SerialPlotter_t *huart, int16_t val) {
    huart->UART_Tx_Data[huart->index++] = (uint8_t)(val & 0xFF);
    huart->UART_Tx_Data[huart->index++] = (uint8_t)((val >> 8) & 0xFF);
}

void Plotter_AppendInt32(SerialPlotter_t *huart, int32_t val) {
    huart->UART_Tx_Data[huart->index++] = (uint8_t)(val & 0xFF);
    huart->UART_Tx_Data[huart->index++] = (uint8_t)((val >> 8) & 0xFF);
    huart->UART_Tx_Data[huart->index++] = (uint8_t)((val >> 16) & 0xFF);
    huart->UART_Tx_Data[huart->index++] = (uint8_t)((val >> 24) & 0xFF);
}

void Plotter_AppendUint8(SerialPlotter_t *huart, uint8_t val) {
    huart->UART_Tx_Data[huart->index++] = val;
}
