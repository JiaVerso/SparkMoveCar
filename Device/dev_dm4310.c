/**
 * @file dev_dm4310.c
 * @author JiaVerso
 * @brief dm4310 设备层文件
 * @version 0.1
 * @date 2026-04-22
 *
 * @copyright 根据达妙官方电机示例代码移植.
 *
 */
#include "drv_dm4310.h"
#include "dev_dm4310.h"
#include "drv_can.h"
#include "drv_math.h"
#include <cstdint>
#include <string.h>

dm_motor_t motor[Motor_Max];
int8_t motor_id = 1;

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	*m:        指向要初始化的电机结构体的指针 
* @retval:     	can_id:    电机硬件实际的 CAN ID
* @param:       init_mode: 初始化的控制模式
* @details:    	初始化DM4310型号的电机，设置默认参数和控制模式。
*               设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm4310_motor_init(dm_motor_t *p, uint8_t id, uint8_t init_mode)
{
    if(p = NULL)
    {
        return;
    }
	// 初始化Motor的电机结构
	memset(p, 0, sizeof(dm_motor_t));

	// 设置Motor的电机信息
	p->id = id + init_mode;
	p->ctrl.mode = init_mode;		// 0: MIT模式   1: 位置速度模式   2: 速度模式
	p->cmd.mode = init_mode;

}
/**
************************************************************************
* @brief:      	motor_para_add: 修改电机参数函数
* @param[in]:   motor:   指向dm_motor_t结构的指针，包含电机参数信息
* @retval:     	void
* @details:    	根据当前LCD标志位（lcd_flag），修改指定电机的参数信息。
*               根据lcd_flag的不同，可修改电机ID、控制模式、位置设定、速度设定、
*               扭矩设定、比例增益和微分增益等参数。
************************************************************************
**/
void motor_para_add(dm_motor_t *motor)
{
	switch(lcd_flag)
	{
		case 0:
			motor_id += 1;
			if (motor_id > num)
				motor_id = 1;
			break;
		case 1:
			motor->ctrl.mode += 1;
			if (motor->ctrl.mode > 2)
				motor->ctrl.mode = 0;
			break;
		case 2:
			motor->cmd.pos_set += 1.0f;
			break;
		case 3:
			motor->cmd.vel_set += 1.0f;
			break;
		case 4:
			motor->cmd.tor_set += 1.0f;
			break;
		case 5:
			motor->cmd.kp_set += 1.0f;
			break;
		case 6:
			motor->cmd.kd_set += 0.5f;
			break;
	}
}
/**
************************************************************************
* @brief:      	motor_para_minus: 减小电机参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机参数信息
* @retval:     	void
* @details:    	根据当前LCD标志位（lcd_flag），减小指定电机的参数信息。
*               根据lcd_flag的不同，可减小电机ID、控制模式、位置设定、速度设定、
*               扭矩设定、比例增益和微分增益等参数。
************************************************************************
**/
void motor_para_minus(dm_motor_t *motor)
{
	switch(lcd_flag)
	{
		case 0:
			motor_id -= 1;
			if (motor_id < 1)
				motor_id = num;
			break;
		case 1:
			motor->ctrl.mode -= 1;
			if (motor->ctrl.mode < 0)
				motor->ctrl.mode = 2;
			break;
		case 2:
			motor->cmd.pos_set -= 1.0f;
			break;
		case 3:
			motor->cmd.vel_set -= 1.0f;
			break;
		case 4:
			motor->cmd.tor_set -= 1.0f;
			break;
		case 5:
			motor->cmd.kp_set -= 1.0f;
			break;
		case 6:
			motor->cmd.kd_set -= 0.5f;
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_enable: 启用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），启用对应的电机控制。
*               设置指定电机的启动标志，并调用dm4310_enable函数启用电机。
************************************************************************
**/
void ctrl_enable(Enum_Motor_Status target_motor)
{
	switch(target_motor)
	{
		case Motor1_Status_ENABLED:
			// 启用Motor1的电机控制
			motor[Motor1].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor1]);
			break;
		case Motor2_Status_ENABLED:
			// 启用Motor2的电机控制
			motor[Motor2].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor2]);
			break;
        case Motor_ALL_Status_ENABLED:
			// 启用所有的电机控制
            motor[Motor1].start_flag = 1;
            dm4310_enable(&hcan1, &motor[Motor1]);
			
            motor[Motor2].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor2]);
			break;
        
	}
}
/**
************************************************************************
* @brief:      	ctrl_disable: 禁用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），禁用对应的电机控制。
*               设置指定电机的启动标志为0，并调用dm4310_disable函数禁用电机。
************************************************************************
**/
void ctrl_disable(Enum_Motor_Status target_motor)
{
	switch(target_motor)
	{
		case Motor1_Status_DISABLED:
			// 禁用Motor1的电机控制
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);
			break;
		case Motor2_Status_DISABLED:
			// 禁用Motor2的电机控制
			motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor2]);
			break;
		case Motor_ALL_Status_DISABLED:
			// 禁用全部的电机控制
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);

            motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor2]);
			break;

	}
}
/**
************************************************************************
* @brief:      	ctrl_set: 设置电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），设置对应电机的参数。
*               调用dm4310_set函数设置指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_set(dm_motor_t *motor)
{
	dm4310_set(motor);
}
/**
************************************************************************
* @brief:      	ctrl_clear_para: 清除电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_para(dm_motor_t *motor)
{
	dm4310_clear_para(motor);
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: 清除电机错误信息
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_err(CAN_HandleTypeDef *hcan, dm_motor_t *motor)
{
	dm4310_clear_err(hcan, motor);
}
/**
************************************************************************
* @brief:      	ctrl_add: 增加电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），增加对应电机的参数。
*               调用motor_para_add函数增加指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_add(dm_motor_t *motor)
{
	motor_para_add(motor);
}
/**
************************************************************************
* @brief:      	ctrl_minus: 减少电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），减少对应电机的参数。
*               调用motor_para_minus函数减少指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_minus(dm_motor_t *motor)
{
	motor_para_minus(motor);
}
/**
************************************************************************
* @brief:      	ctrl_send: 发送电机控制命令函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），向对应电机发送控制命令。
*               调用dm4310_ctrl_send函数向指定电机发送控制命令，以响应外部命令。
************************************************************************
**/
void ctrl_send(CAN_HandleTypeDef *hcan, dm_motor_t *motor)
{
	dm4310_ctrl_send(hcan, motor);
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0: 
			{
				switch ((rx_data[0])&0x0F)
				{
					case 11: dm4310_fbdata(&motor[Motor1], rx_data); break;
					case 12: dm4310_fbdata(&motor[Motor3], rx_data); break;
				}
				
			} break;
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN2接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
uint16_t rec_id;
void can2_rx_callback(void)
{
	
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan2, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0: dm4310_fbdata(&motor[Motor2], rx_data); break;
	}
}

