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

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}