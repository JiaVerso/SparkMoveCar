#ifndef __DRV_MATH_H__
#define __DRV_MATH_H__

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f4xx_hal.h"

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif /*  __DRV_MATH_H__ */