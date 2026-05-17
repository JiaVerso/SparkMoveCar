/**
 * @file chassis.c
 * @author JiaVerso 
 * @brief 美国手：1. CH1: 右摇杆左右 -> 转向 wz  
                2. CH2: 右摇杆上下 -> 前进/后退 vx  
                3. CH3: 左摇杆上下 -> 油门/速度倍率 throttle  
                4. CH4: 左摇杆左右 -> 暂时不用
                5. CH8: 急停开关
 * @version 0.1
 * @date 2026-5-13
 *
 * @copyright Copyright (c) 2026 JiaVerso
 *
 */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>
#include "drv_can.h"
#include "motor_dji.h"
#include "pidcontroller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALL_MOTORS 0x00
#define ALL_MOTOR_COUNT 6U

#define CHASSIS_MOTOR_COUNT 4U

// 遥控输入通道数量  Number of remote control input channels
#define CHASSIS_SBUS_CH_COUNT 8U
#define CHASSIS_SBUS_VX_CH    1U
#define CHASSIS_SBUS_WZ_CH    0U
#define CHASSIS_SBUS_DM_RADPS_CH   2U

// SBUS输入的最小值、中心值、最大值和死区  Minimum, center, maximum, and deadband for remote control input
#define CHASSIS_SBUS_MIN      300U
#define CHASSIS_SBUS_CENTER   1000U
#define CHASSIS_SBUS_MAX      1700U
#define CHASSIS_SBUS_DEADBAND 0.10f

// 底盘最大速度限制  Maximum speed limits for the chassis
#define CHASSIS_MAX_VX_MPS      1.0f
#define CHASSIS_MAX_WZ_RADPS    1.0f

// 轮子直径 Wheel diameter 6inch = 0.1524m
#define CHASSIS_WHEEL_DIAMETER_M 0.1524f
// 轴距，前后轮中心之间的距离 Wheelbase, the distance between the centers of the front and rear wheels
#define CHASSIS_WHEELBASE_M      0.500f
// 轮距，左右轮中心之间的距离 Track width, the distance between the centers of the left and right wheels
#define CHASSIS_TRACK_WIDTH_M    0.48f
// 最大转向角，单位是弧度 Maximum steering angle in radians
#define CHASSIS_MAX_STEER_RAD    0.610f

#define CHASSIS_VESC_POLE_PAIRS  7.0f
// VESC的减速比，电机转速 / 轮子转速  The gear ratio for VESC, motor speed / wheel speed
#define CHASSIS_VESC_GEAR_RATIO  19.203f
#define CHASSIS_C620_GEAR_RATIO  19.203f

// 电流限制 Current limits
#define CHASSIS_VESC_CURRENT_LIMIT_A  5.0f
// C620电机的电流限制，单位是mA  Current limit for C620 motors, in mA
#define CHASSIS_C620_CURRENT_LIMIT    5000.0f

// 电机类型枚举  Motor type enumeration
typedef enum {
    CHASSIS_WHEEL_LF = 0,
    CHASSIS_WHEEL_RF,
    CHASSIS_WHEEL_LB,
    CHASSIS_WHEEL_RB,
#if ALL_MOTORS
    CHASSIS_WHEEL_LM,
    CHASSIS_WHEEL_RM,
#endif
} ChassisWheel_e;

typedef enum {
    CHASSIS_MOTOR_TYPE_VESC = 0,
    CHASSIS_MOTOR_TYPE_C620,
} ChassisMotorType_e;

// 底盘模式枚举  Chassis mode enumeration
typedef enum {
    CHASSIS_LOCKED = 0,
    CHASSIS_ARMED,
    CHASSIS_ESTOP,
} ChassisMode_e;

// 电机控制结构体，包含电机的类型、CAN接口、ID、PID参数等  The motor control structure, including the motor type, CAN interface, ID, PID parameters, etc.
typedef struct {
    ChassisWheel_e wheel;
    ChassisMotorType_e type;

    CAN_HandleTypeDef *hcan;
    uint8_t vesc_id;

    uint32_t c620_rx_id;
    uint32_t c620_tx_id;
    uint8_t c620_tx_slot;

    // 将电机反馈的转速转换成轮子转速的比例系数  The ratio coefficient to convert the motor feedback speed into wheel speed
    float feedback_to_wheel_rpm;
    float command_direction;
    float current_limit;

    // PID parameters  PID参数
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_kf;
    float pid_max_integral;

    Motor_t c620_motor;
    PID_t speed_pid;

    // 反馈的轮子转速  Feedback wheel speed
    float target_wheel_rpm;
    float feedback_wheel_rpm;
    float current_cmd;
} ChassisMotor_t;

extern ChassisMotor_t ChassisMotor_Table[CHASSIS_MOTOR_COUNT];

void ChassisMotor_InitAll(void);
void ChassisMotor_CANRxDispatch(Struct_CAN_Rx_Buffer *rx_buffer);

float ChassisMotor_GetFeedbackRpm(ChassisWheel_e wheel);
void ChassisMotor_SetWheelTargetRpm(ChassisWheel_e wheel, float wheel_rpm);
void ChassisMotor_SetChassisSpeed(float vx_mps, float wz_radps);
void ChassisMotor_UpdateFromSbusChannels(const uint16_t channels[CHASSIS_SBUS_CH_COUNT]);

void ChassisMotor_ControlLoop(void);
void ChassisMotor_SendCurrent(ChassisMotor_t *motor);
void ChassisMotor_SendAllCurrent(void);
void ChassisMotor_SendAllCurrent_Section(void);

#ifdef __cplusplus
}
#endif

#endif



