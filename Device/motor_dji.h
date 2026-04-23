#ifndef __MOTOR_DJI_H
#define __MOTOR_DJI_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_MOTORS 6    // 可配置电机最大数量

#define DJI_MOTOR_LF     0   // 左前
#define DJI_MOTOR_RF     1   // 右前
#define DJI_MOTOR_LM     2   // 左中
#define DJI_MOTOR_RM     3   // 右中
#define DJI_MOTOR_LB     4   // 左后
#define DJI_MOTOR_RB     5   // 右后

#define CAN_RX_ID_LF     0x201  // 左前
#define CAN_RX_ID_RF     0x202  // 右前
#define CAN_RX_ID_LM     0x203  // 左中
#define CAN_RX_ID_RM     0x204  // 右中
#define CAN_RX_ID_LB     0x205  // 左后
#define CAN_RX_ID_RB     0x206  // 右后

#define DM4310_MOTOR     0x00  // 右后

// 电机类定义
typedef struct {
    uint32_t rx_can_id;   // 接收报文 ID
    uint32_t tx_can_id;   // 发送控制 ID
    
    // 状态数据
    int16_t rx_angle;
    int16_t rx_speed;
    int16_t rx_torque;
    int16_t rx_temperature; 
    
    // 控制数据
    int16_t target_current; // 要下发的控制电流
    
    // 缓冲区
    uint8_t rx_buffer[8];   
    uint8_t tx_buffer[8];   
} Motor_t;

typedef struct {
    uint32_t motor_id;
    uint32_t tx_can_id;
}Motor_Config_t;

// Motor 管理表
extern const Motor_Config_t Motor_Config_Table[MAX_MOTORS];

// 电机对象方法
void Motor_Init(Motor_t *motor, uint32_t rx_id, uint32_t tx_id);
void Motor_ParseRxData(Motor_t *motor, uint8_t *rx_data); // 解析接收
void Motor_PackTxData(Motor_t *motor);                    // 打包发送数据

// 多电机管理器方法
void MotorManager_Register(Motor_t *motor); // 电机注册表
Motor_t* MotorManager_Find(uint32_t rx_id); 

#endif