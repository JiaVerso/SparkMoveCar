#include "motor_dji.h"
#include <stddef.h>

#define MAX_MOTORS 8
static Motor_t* Motor_Registry[MAX_MOTORS]; // 电机指针数组 (注册表)
static uint8_t Motor_Count = 0;

// 将电机注册到系统中
void MotorManager_Register(Motor_t *motor) {
    if (Motor_Count < MAX_MOTORS && motor != NULL) {
        Motor_Registry[Motor_Count++] = motor;
    }
}

// O(N) 查找，根据收到的 CAN ID 找到是哪个电机
Motor_t* MotorManager_Find(uint32_t rx_id) {
    for (int i = 0; i < Motor_Count; i++) {
        if (Motor_Registry[i]->rx_can_id == rx_id) {
            return Motor_Registry[i];
        }
    }
    return NULL;
}

// 打包发送数据到自身的 Tx Buffer
void Motor_PackTxData(Motor_t *motor) {
    // 假设大疆 M3508 协议：前两个字节是电机的控制电流 (高位在前)
    motor->tx_buffer[0] = (motor->target_current >> 8) & 0xFF;
    motor->tx_buffer[1] = motor->target_current & 0xFF;
    // 其他字节清零...
}