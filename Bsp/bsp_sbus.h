#ifndef BSP_SBUS_H_
#define BSP_SBUS_H_

#include "stdint.h"
#include "drv_uart.h"
#include "usart.h"
#include <stdbool.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS       0x00
#define MANUAL 0x00

#define SBUS_MAX_VALUE  (1700)
#define SBUS_MIN_VALUE  (300)

// SBUS data structure  SBUS数据结构体
typedef struct _sbus_data_t {
    int16_t sbus_speed_set[4];
}sbus_data_t;

typedef struct Motion_Parameters {
    float vx;
    float vy;
    float vz;
    float throttle;
}move;

void SBUS_DMA_Start(UART_HandleTypeDef *huart);
void SBUS_Receive(uint8_t data);

bool SBUS_GetChannels(uint16_t out[], uint16_t len);
bool SBUS_IsFailsafe(void);
bool SBUS_HasNewFrame(void);
void SBUS_ClearNewFrame(void);

void SBUS_Handle(void);


// void Linear_Mapping(uint16_t g_sbus_channels[]);

#endif /* BSP_SBUS_H_ */
