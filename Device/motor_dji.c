#include "motor_dji.h"
#include <stddef.h>
#include <string.h>
#include "fifo.h"

static Motor_t* Motor_Registry[MAX_MOTORS]; // 电机指针数组 (注册表)
static uint8_t Motor_Count = 0;
Motor_t DriveMotors[MAX_MOTORS];

const Motor_Config_t Motor_Config_Table[MAX_MOTORS] = {
    {DJI_MOTOR_LF, CAN_RX_ID_LF},
    {DJI_MOTOR_RF, CAN_RX_ID_RF},
    {DJI_MOTOR_LM, CAN_RX_ID_LM},
    {DJI_MOTOR_RM, CAN_RX_ID_RM},
    {DJI_MOTOR_LB, CAN_RX_ID_LB},
    {DJI_MOTOR_RB, CAN_RX_ID_RB}
};

// 单个电机初始化
void Motor_Init(Motor_t *motor, uint32_t rx_id, uint32_t tx_id){
    // 安全检查
    if (motor == NULL) return;

    // 绑定 CAN ID
    motor->rx_can_id = rx_id;
    motor->tx_can_id = tx_id;
    // 初始化参数
    motor->rx_angle = 0;
    motor->rx_speed = 0;
    motor->rx_torque = 0;
    motor->rx_temperature = 0;
    motor->target_current = 0;

    // 初始化收发缓冲区
    memset(motor->rx_buffer, 0, sizeof(motor->rx_buffer));
    memset(motor->tx_buffer, 0, sizeof(motor->tx_buffer));
}

// 将电机注册到系统中
void MotorManager_Register(Motor_t *motor) {
    if (Motor_Count < MAX_MOTORS && motor != NULL) {
        Motor_Registry[Motor_Count++] = motor;
    }
}

// 初始化所有的电机
void All_DriveMotors_Init(void) {

       
    for (int i = 0; i < MAX_MOTORS; i++) {

        uint32_t rx_motor_id = Motor_Config_Table[i].tx_can_id; 
        uint32_t tx_motor_id = 0x200; 
        // 1-4 号电机
        if (rx_motor_id >= 0x201 && rx_motor_id <= 0x204) {
            tx_motor_id = 0x200; 
        } 
        // 5-8 号电机
        else if (rx_motor_id >= 0x205 && rx_motor_id <= 0x208) {
            tx_motor_id = 0x1FF; 
        } 
        // 错误检查
        else {
            tx_motor_id = 0x000; // ID 错误
        }
        
        Motor_Init(&DriveMotors[i], rx_motor_id + i, tx_motor_id);
        
        // 将每个电机注册到统一管理器中
        MotorManager_Register(&DriveMotors[i]);
    }
}

// 注册表查找，根据收到的 CAN ID 找到是哪个电机
Motor_t* MotorManager_Find(uint32_t rx_id) {
    for (int i = 0; i < Motor_Count; i++) {
        if (Motor_Registry[i]->rx_can_id == rx_id) {
            return Motor_Registry[i];
        }
    }
    return NULL;
}

/**
 * @brief 解析M3508电机电调发回来的 CAN 报文
 * @param motor   目标电机对象指针
 * @param rx_data CAN 接收中断里拿到的 8 字节数据
 */
void Motor_ParseRxData(Motor_t *motor, uint8_t *rx_data) 
{
    // 安全校验
    if (motor == NULL || rx_data == NULL) {
        return; 
    }
    // 类型强转（整型）
    // 解析机械角度 (0 ~ 8191)
    motor->rx_angle = (uint16_t)((rx_data[0] << 8) | rx_data[1]); 
    
    // 解析转速
    motor->rx_speed = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    
    // 解析实际转矩电流
    motor->rx_torque = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    
    // 解析电机温度
    motor->rx_temperature = rx_data[6]; 

}

// 打包发送数据到自身的 Tx Buffer
void Motor_PackTxData(Motor_t *motor) {

    motor->tx_buffer[0] = (motor->target_current >> 8) & 0xFF;
    motor->tx_buffer[1] = (motor->target_current & 0xFF);
}