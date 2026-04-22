/**
 * @file drv_math.c
 * @author JiaVerso
 * @brief BSP层
 * @version 0.1
 * @date 2026-04-15
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */
#include "drv_math.h"
#include <stddef.h>

// 用于拆分 float 的联合体工具
typedef union {
    float float_val;
    uint8_t bytes[4];
} FloatConverter_t;

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
