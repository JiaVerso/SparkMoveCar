/**
 * @file chassis.c
 * @author JiaVerso 
 * @brief 负责：1. 遥控器输入转 vx/wz  2. 四轮差速运动学  3.底盘控制周期入口
 * @version 0.1
 * @date 2026-5-13
 *
 * @copyright Copyright (c) 2026 JiaVerso
 *
 */

#include "chassis.h"

#include <stddef.h>
#include "dev_n630.h"

#define DRIVE_PI 3.1415926f

// 将数值限制在[-limit, limit]范围内  Clamp a value to the range [-limit, limit]
static float ChassisMotor_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// 线性化遥控输入，转换成[-1.0, 1.0]范围的速度命令  Linearize remote control input into speed commands in the range [-1.0, 1.0]
static float ChassisMotor_NormalizeChannel(uint16_t channel)
{
    float value;

    if (channel >= CHASSIS_SBUS_CH_COUNT) {
        value = (float)(channel - CHASSIS_SBUS_CENTER) /
                (float)(CHASSIS_SBUS_MAX - CHASSIS_SBUS_CENTER);
    } else {
        value = -((float)(CHASSIS_SBUS_CENTER - channel) /
                  (float)(CHASSIS_SBUS_CENTER - CHASSIS_SBUS_MIN));
    }

    value = ChassisMotor_Clamp(value, 1.0f);
    if (value > -CHASSIS_SBUS_DEADBAND && value < CHASSIS_SBUS_DEADBAND) {
        value = 0.0f;
    }
    return value;
}

static ChassisMotor_t *ChassisMotor_FindByWheel(ChassisWheel_e wheel)
{
    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (ChassisMotor_Table[i].wheel == wheel) {
            return &ChassisMotor_Table[i];
        }
    }
    return NULL;
}

static ChassisMotor_t *ChassisMotor_FindC620ByStdId(uint32_t std_id)
{
    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (ChassisMotor_Table[i].type == CHASSIS_MOTOR_TYPE_C620 &&
            ChassisMotor_Table[i].c620_rx_id == std_id) {
            return &ChassisMotor_Table[i];
        }
    }
    return NULL;
}

ChassisMotor_t ChassisMotor_Table[CHASSIS_MOTOR_COUNT] = {
    {
        .wheel = CHASSIS_WHEEL_LF,
        .type = CHASSIS_MOTOR_TYPE_VESC,
        .hcan = &hcan1,
        .vesc_id = 21,
        .feedback_to_wheel_rpm = 1.0f / (CHASSIS_VESC_POLE_PAIRS * CHASSIS_VESC_GEAR_RATIO),
        .command_direction = 1.0f,
        .current_limit = CHASSIS_VESC_CURRENT_LIMIT_A,
        .pid_kp = 0.02f,
        .pid_ki = 0.001f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 5.0f,
    },
    {
        .wheel = CHASSIS_WHEEL_RF,
        .type = CHASSIS_MOTOR_TYPE_VESC,
        .hcan = &hcan1,
        .vesc_id = 22,
        .feedback_to_wheel_rpm = 1.0f / (CHASSIS_VESC_POLE_PAIRS * CHASSIS_VESC_GEAR_RATIO),
        .command_direction = 1.0f,
        .current_limit = CHASSIS_VESC_CURRENT_LIMIT_A,
        .pid_kp = 0.02f,
        .pid_ki = 0.001f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 5.0f,
    },
    {
        .wheel = CHASSIS_WHEEL_LB,
        .type = CHASSIS_MOTOR_TYPE_VESC,
        .hcan = &hcan1,
        .vesc_id = 24,
        .feedback_to_wheel_rpm = 1.0f / (CHASSIS_VESC_POLE_PAIRS * CHASSIS_VESC_GEAR_RATIO),
        .command_direction = 1.0f,
        .current_limit = CHASSIS_VESC_CURRENT_LIMIT_A,
        .pid_kp = 0.02f,
        .pid_ki = 0.001f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 5.0f,
    },
    {
        .wheel = CHASSIS_WHEEL_RB,
        .type = CHASSIS_MOTOR_TYPE_C620,
        .hcan = &hcan1,
        .c620_rx_id = 0x202,
        .c620_tx_id = 0x200,
        .c620_tx_slot = 1,
        .feedback_to_wheel_rpm = 1.0f / CHASSIS_C620_GEAR_RATIO,
        .command_direction = 1.0f,
        .current_limit = CHASSIS_C620_CURRENT_LIMIT,
        .pid_kp = 4.0f,
        .pid_ki = 0.02f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 1500.0f,
    },
};

void ChassisMotor_InitAll(void)
{
    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        ChassisMotor_t *motor = &ChassisMotor_Table[i];

        PID_Init(&motor->speed_pid,
                 motor->pid_kp,
                 motor->pid_ki,
                 motor->pid_kd,
                 motor->pid_kf,
                 motor->pid_max_integral,
                 motor->current_limit);

        motor->target_wheel_rpm = 0.0f;
        motor->feedback_wheel_rpm = 0.0f;
        motor->current_cmd = 0.0f;

        if (motor->type == CHASSIS_MOTOR_TYPE_C620) {
            Motor_Init(&motor->c620_motor, motor->c620_rx_id, motor->c620_tx_id);
        }
    }
}

void ChassisMotor_CANRxDispatch(Struct_CAN_Rx_Buffer *rx_buffer)
{
    if (rx_buffer == NULL) {
        return;
    }

    if (rx_buffer->Header.IDE == CAN_ID_STD) {
        ChassisMotor_t *motor = ChassisMotor_FindC620ByStdId(rx_buffer->Header.StdId);
        if (motor != NULL) {
            Motor_ParseRxData(&motor->c620_motor, rx_buffer->Data);
            motor->feedback_wheel_rpm = ChassisMotor_GetFeedbackRpm(motor->wheel);
        }
    } else {
        Motor_UpdateData(rx_buffer->Header.ExtId, rx_buffer->Data);
    }
}

float ChassisMotor_GetFeedbackRpm(ChassisWheel_e wheel)
{
    ChassisMotor_t *motor = ChassisMotor_FindByWheel(wheel);
    float raw_rpm = 0.0f;

    if (motor == NULL) {
        return 0.0f;
    }

    if (motor->type == CHASSIS_MOTOR_TYPE_VESC) {
        raw_rpm = n630_motor[motor->vesc_id].rpm;
    } else {
        raw_rpm = (float)motor->c620_motor.rx_speed;
    }

    return raw_rpm * motor->feedback_to_wheel_rpm * motor->command_direction;
}

void ChassisMotor_SetWheelTargetRpm(ChassisWheel_e wheel, float wheel_rpm)
{
    ChassisMotor_t *motor = ChassisMotor_FindByWheel(wheel);

    if (motor != NULL) {
        motor->target_wheel_rpm = wheel_rpm;
    }
}

void ChassisMotor_SetChassisSpeed(float vx_mps, float wz_radps)
{
    const float wheel_circumference_m = DRIVE_WHEEL_DIAMETER_M * DRIVE_PI;
    const float left_mps = vx_mps - wz_radps * (DRIVE_TRACK_WIDTH_M * 0.5f);
    const float right_mps = vx_mps + wz_radps * (DRIVE_TRACK_WIDTH_M * 0.5f);
    const float left_rpm = left_mps * 60.0f / wheel_circumference_m;
    const float right_rpm = right_mps * 60.0f / wheel_circumference_m;

    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_LF, left_rpm);
    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_LB, left_rpm);
    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_RF, right_rpm);
    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_RB, right_rpm);
}

void ChassisMotor_UpdateFromSbusChannels(const uint16_t channels[CHASSIS_REMOTE_CH_COUNT])
{
    float vx_cmd;
    float wz_cmd;

    if (channels == NULL) {
        ChassisMotor_SetChassisSpeed(0.0f, 0.0f);
        return;
    }

    vx_cmd = ChassisMotor_NormalizeChannel(channels[CHASSIS_REMOTE_VX_CH]) * CHASSIS_MAX_VX_MPS;
    wz_cmd = ChassisMotor_NormalizeChannel(channels[CHASSIS_REMOTE_WZ_CH]) * CHASSIS_MAX_WZ_RADPS;

    ChassisMotor_SetChassisSpeed(vx_cmd, wz_cmd);
}

void ChassisMotor_ControlLoop(void)
{
    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        ChassisMotor_t *motor = &ChassisMotor_Table[i];

        motor->feedback_wheel_rpm = ChassisMotor_GetFeedbackRpm(motor->wheel);
        motor->current_cmd = PID_Calculate(&motor->speed_pid,
                                            motor->feedback_wheel_rpm,
                                            motor->target_wheel_rpm);
        motor->current_cmd = ChassisMotor_Clamp(motor->current_cmd, motor->current_limit);
    }

    ChassisMotor_SendAllCurrent();
}

void ChassisMotor_SendCurrent(ChassisMotor_t *motor)
{
    int16_t c620_current;
    uint8_t data[8] = {0};
    uint8_t offset;

    if (motor == NULL) {
        return;
    }

    if (motor->type == CHASSIS_MOTOR_TYPE_VESC) {
        comm_can_set_current(motor->vesc_id,
                             motor->current_cmd * motor->command_direction);
        return;
    }

    c620_current = (int16_t)ChassisMotor_Clamp(motor->current_cmd * motor->command_direction,
                                             motor->current_limit);
    offset = (uint8_t)(motor->c620_tx_slot * 2U);
    data[offset] = (uint8_t)(c620_current >> 8);
    data[offset + 1U] = (uint8_t)c620_current;
    CAN_Send_Data(motor->hcan, (uint16_t)motor->c620_tx_id, data, 8);
}

void ChassisMotor_SendAllCurrent(void)
{
    uint8_t can1_0x200_data[8] = {0};
    uint8_t can1_0x1ff_data[8] = {0};
    uint8_t send_can1_0x200 = 0U;
    uint8_t send_can1_0x1ff = 0U;

    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        ChassisMotor_t *motor = &ChassisMotor_Table[i];

        if (motor->type == CHASSIS_MOTOR_TYPE_VESC) {
            ChassisMotor_SendCurrent(motor);
            continue;
        }

        int16_t current = (int16_t)ChassisMotor_Clamp(motor->current_cmd * motor->command_direction,
                                                    motor->current_limit);
        uint8_t offset = (uint8_t)(motor->c620_tx_slot * 2U);
        uint8_t *data = (motor->c620_tx_id == 0x1FFU) ? can1_0x1ff_data : can1_0x200_data;

        data[offset] = (uint8_t)(current >> 8);
        data[offset + 1U] = (uint8_t)current;

        if (motor->c620_tx_id == 0x1FFU) {
            send_can1_0x1ff = 1U;
        } else {
            send_can1_0x200 = 1U;
        }
    }

    if (send_can1_0x200 != 0U) {
        CAN_Send_Data(&hcan1, 0x200, can1_0x200_data, 8);
    }
    if (send_can1_0x1ff != 0U) {
        CAN_Send_Data(&hcan1, 0x1FF, can1_0x1ff_data, 8);
    }
}
