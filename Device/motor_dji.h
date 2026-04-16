#ifndef __MOTOR_DJI_H
#define __MOTOR_DJI_H

#include <stdint.h>
#include <stdbool.h>

// 电机类定义
typedef struct {
    uint32_t rx_can_id;   // 接收报文 ID
    uint32_t tx_can_id;   // 发送控制 ID
    
    // 状态数据 (从 Rx 报文中解析)
    int16_t rx_encoder
    int16_t rx_omega
    int16_t speed;
    int16_t current;
    
    // 控制数据
    int16_t target_current; // 要下发的控制电流
    
    // 缓冲区 (高内聚：每个电机拥有自己的收发缓存)
    uint8_t rx_buffer[8];   
    uint8_t tx_buffer[8];   
} Motor_t;

// 2. 电机对象方法
void Motor_Init(Motor_t *motor, uint32_t rx_id, uint32_t tx_id);
void Motor_ParseRxData(Motor_t *motor, uint8_t *rx_data); // 解析接收
void Motor_PackTxData(Motor_t *motor);                    // 打包发送数据

// 3. 多电机管理器方法
void MotorManager_Register(Motor_t *motor);
Motor_t* MotorManager_Find(uint32_t rx_id); // 根据 CAN ID 查找对应电机实例

#endif