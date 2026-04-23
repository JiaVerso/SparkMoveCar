#ifndef __DRV_MATH_H__
#define __DRV_MATH_H__

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "drv_uart.h"
#include <math.h>
#include "stm32f4xx_hal.h"

// 转换数据类型
void Plotter_AppendFloat(SerialPlotter_t *huart, float val);
void Plotter_AppendInt32(SerialPlotter_t *huart, int32_t val);
void Plotter_AppendInt16(SerialPlotter_t *huart, int16_t val);
void Plotter_AppendUint8(SerialPlotter_t *huart, uint8_t val);

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

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


#endif /*  __DRV_MATH_H__ */