#ifndef __DRV_DM4310_H__
#define __DRV_DM4310_H__
#include "main.h"
#include "drv_math.h"
#include "drv_can.h"
#include "motor_dji.h"


#define MIT_MODE 		0x000
#define POS_MODE		0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

// 电机回传信息结构体（反馈）
typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;

// 电机参数设置结构体（控制）
typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

// 多电机结构体
typedef struct
{
	int8_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
	motor_ctrl_t cmd;       // 指令和控制分开，可以在控制前对指令数值进行检查
}dm_motor_t;

void dm4310_ctrl_send(CAN_HandleTypeDef *hcan, dm_motor_t *motor);
void dm4310_enable(CAN_HandleTypeDef *hcan, dm_motor_t *motor);
void dm4310_disable(CAN_HandleTypeDef *hcan, dm_motor_t *motor);
void dm4310_set(dm_motor_t *motor);
void dm4310_clear_para(dm_motor_t *motor);
void dm4310_clear_err(CAN_HandleTypeDef *hcan, dm_motor_t *motor);
void dm4310_fbdata(dm_motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(CAN_HandleTypeDef *hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float _vel);
void save_pos_zero(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);

#endif /*  __DRV_DM4310_H__ */