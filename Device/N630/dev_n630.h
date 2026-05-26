#ifndef __DEV_N630_H
#define __DEV_N630_H

#include "stm32f4xx_hal.h"

typedef enum {
    // 控制命令
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,       // 刹车电流
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_SET_CURRENT_REL = 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
    // 反馈状态
    CAN_PACKET_STATUS = 9,
    CAN_PACKET_STATUS_2 = 14,
    CAN_PACKET_STATUS_3 = 15,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_STATUS_5 = 27,
} CAN_PACKET_ID;
// 枚举

// 电机状态结构体
typedef struct {
    uint8_t id;
    float rpm;          // 转速
    float current;      // 电流
    float duty;         // 占空比 -1.0 ~ 1.0
    float amp_hours;    
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    float temp_fet;     // 功率管温度
    float temp_motor;   // 电机温度
    float current_in;   // 输入电流
    float pid_pos;      // 位置环PID的目标位置
    float tachometer;   // 转速表
    float input_voltage;// 输入电压
} N630_Motor_t;

extern N630_Motor_t n630_motor[30];

void can_transmit_eid(CAN_HandleTypeDef *hcan, uint32_t id, const uint8_t *data, uint8_t len);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
void comm_can_set_duty(CAN_HandleTypeDef *hcan, uint8_t controller_id, float duty);
void comm_can_set_current(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current);
void comm_can_set_current_off_delay(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current, float off_delay);
void comm_can_set_current_brake(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current);
void comm_can_set_rpm(CAN_HandleTypeDef *hcan, uint8_t controller_id, float rpm);
void comm_can_set_pos(CAN_HandleTypeDef *hcan, uint8_t controller_id, float pos);
void comm_can_set_current_rel(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current_rel);
void comm_can_set_current_rel_off_delay(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current_rel, float off_delay);
void comm_can_set_current_brake_rel(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current_rel);
void Motor_UpdateData(uint32_t ext_id, uint8_t *pData);


#endif