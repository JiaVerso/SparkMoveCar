#ifndef __DEV_DM4310_H__
#define __DEV_DM4310_H__
#include "main.h"
#include "drv_math.h"
#include "drv_dm4310.h"
#include "motor_dji.h"

#define MOTOR_LEFT_CANID 0X01

extern int8_t motor_id;

typedef enum
{
	Motor1,
	Motor2,
	Motor_Max
} dm_motor_num;

typedef enum 
{
    Motor1_Status_ENABLED = 0,
    Motor2_Status_ENABLED,
    Motor_ALL_Status_ENABLED,

    Motor1_Status_DISABLED,
    Motor2_Status_DISABLED,
    Motor_ALL_Status_DISABLED
}Enum_Motor_Status;

extern dm_motor_t motor[Motor_Max];

void dm4310_motor_init(CAN_HandleTypeDef *hcan, dm_motor_t *p, uint8_t id, uint16_t init_mode);
void ctrl_enable(Enum_Motor_Status target_motor);
void ctrl_disable(Enum_Motor_Status target_motor);
void ctrl_set(dm_motor_t *motor);
void ctrl_clear_para(dm_motor_t *motor);
void ctrl_clear_err(CAN_HandleTypeDef *hcan, dm_motor_t *motor);
void ctrl_add(dm_motor_t *motor);
void ctrl_minus(dm_motor_t *motor);
void ctrl_send(CAN_HandleTypeDef *hcan, dm_motor_t *motor);

// void can1_rx_callback(void);
// void can2_rx_callback(void);

#endif /* __DEV_DM4310_H__ */

