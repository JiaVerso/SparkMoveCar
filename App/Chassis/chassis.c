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
#include <stdint.h>
#include "dev_n630.h"
#include "drv_can.h"
#include "stm32f4xx_hal_can.h"

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

    if (channel >= CHASSIS_SBUS_CENTER) {
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

// 根据轮子枚举查找对应的电机控制结构体指针  Find the corresponding motor control structure pointer based on the wheel enumeration
static ChassisMotor_t *ChassisMotor_FindByWheel(ChassisWheel_e wheel)
{
    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (ChassisMotor_Table[i].wheel == wheel) {
            return &ChassisMotor_Table[i];
        }
    }
    return NULL;
}

// 根据CAN ID查找对应的C620电机控制结构体指针  Find the corresponding C620 motor control structure pointer based on the CAN ID
static ChassisMotor_t *ChassisMotor_FindC620ByStdId(uint32_t std_id)
{
    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (ChassisMotor_Table[i].type == CHASSIS_MOTOR_TYPE_C620 && ChassisMotor_Table[i].c620_rx_id == std_id) {
            return &ChassisMotor_Table[i];
        }
    }
    return NULL;
}

// 根据CAN ID查找对应的VESC电机控制结构体指针  Find the corresponding C620 motor control structure pointer based on the CAN ID
static ChassisMotor_t *ChassisMotor_FindVescByExtId(uint32_t ext_id)
{
    uint8_t vesc_id = ext_id & 0xFF;

    for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (ChassisMotor_Table[i].type == CHASSIS_MOTOR_TYPE_VESC &&
            ChassisMotor_Table[i].vesc_id == vesc_id) {
            return &ChassisMotor_Table[i];
        }
    }

    return NULL;
}

ChassisMotor_t ChassisMotor_Table[CHASSIS_MOTOR_COUNT] = {
    // 左前轮参数设置  Parameters for left front wheel
    {
        .wheel = CHASSIS_WHEEL_LF,
        .type = CHASSIS_MOTOR_TYPE_VESC,
        .hcan = &hcan1,
        .vesc_id = 22,

        // 将电机反馈的转速转换成轮子转速的比例系数  The ratio coefficient to convert the motor feedback speed into wheel speed
        .feedback_to_wheel_rpm = 1.0f / (CHASSIS_VESC_POLE_PAIRS * CHASSIS_VESC_GEAR_RATIO),
        .command_direction = -1.0f,
        .current_limit = CHASSIS_VESC_CURRENT_LIMIT_A,
          
        // VESC PID output uint: ampere(A)
        .pid_kp = 0.04f,
        .pid_ki = 0.001f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 10.0f,
    },
    {
        .wheel = CHASSIS_WHEEL_RF,
        .type = CHASSIS_MOTOR_TYPE_VESC,
        .hcan = &hcan1,
        .vesc_id = 21,
        .feedback_to_wheel_rpm = 1.0f / (CHASSIS_VESC_POLE_PAIRS * CHASSIS_VESC_GEAR_RATIO),
        .command_direction = 1.0f,
        .current_limit = CHASSIS_VESC_CURRENT_LIMIT_A,
          
        // VESC PID output uint: ampere(A)
        .pid_kp = 0.04f,
        .pid_ki = 0.001f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 10.0f,
    },
    {
        .wheel = CHASSIS_WHEEL_LB,
        .type = CHASSIS_MOTOR_TYPE_VESC,
        .hcan = &hcan1,
        .vesc_id = 24,
        .feedback_to_wheel_rpm = 1.0f / (CHASSIS_VESC_POLE_PAIRS * CHASSIS_VESC_GEAR_RATIO),
        .command_direction = -1.0f,
        .current_limit = CHASSIS_VESC_CURRENT_LIMIT_A,

        // VESC PID output uint: ampere(A)
        .pid_kp = 0.04f,
        .pid_ki = 0.001f,
        .pid_kd = 0.0f,
        .pid_kf = 0.0f,
        .pid_max_integral = 10.0f,
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
        // C620 PID output unit: DJI current command raw value, in the range of [-65535, 65535], corresponding to [-max_current, max_current]
        .pid_kp = 30.0f,
        .pid_ki = 0.2f,
        .pid_kd = 0.0f,
        .pid_kf = 5.0f,
        .pid_max_integral = 1200.0f,
    },
};

// 初始化所有电机的PID参数和状态  Initialize PID parameters and state for all motors
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

// CAN 回调处理函数，根据收发到的帧类型去执行对应的处理逻辑  CAN callback handler function, execute corresponding processing logic based on the type of frame received
void ChassisMotor_CANRxDispatch(Struct_CAN_Rx_Buffer *rx_buffer)
{
    if (rx_buffer == NULL) {
        return;
    }

    if (rx_buffer->Header.IDE == CAN_ID_STD) {
        ChassisMotor_t *motor = ChassisMotor_FindC620ByStdId(rx_buffer->Header.StdId);
        if (motor != NULL) {
            Motor_ParseRxData(&motor->c620_motor, rx_buffer->Data);

            // 将电机反馈的转速转换成轮子转速  Convert the motor feedback speed into wheel speed
            motor->feedback_wheel_rpm = ChassisMotor_GetFeedbackRpm(motor->wheel);
        }
    } 
    else if (rx_buffer->Header.IDE == CAN_ID_EXT) {
        // 
        ChassisMotor_t *motor = ChassisMotor_FindVescByExtId(rx_buffer->Header.ExtId);
        if(motor == NULL) {
            return;
        }
        Motor_UpdateData(rx_buffer->Header.ExtId, rx_buffer->Data);

        motor->feedback_wheel_rpm = ChassisMotor_GetFeedbackRpm(motor->wheel);
    }
}

// 获取轮子反馈的转速  Get the feedback speed of the wheel
float ChassisMotor_GetFeedbackRpm(ChassisWheel_e wheel)
{
    ChassisMotor_t *motor = ChassisMotor_FindByWheel(wheel);
    float raw_rpm = 0.0f;

    if (motor == NULL) {
        return 0.0f;
    }

    if (motor->type == CHASSIS_MOTOR_TYPE_VESC) {
        // erpm 是电机转速乘以极对数，单位是 rpm * pole_pairs  erpm is the motor speed multiplied by the number of pole pairs, in units of rpm *
        raw_rpm = n630_motor[motor->vesc_id].rpm;
    } else {
        // C620 的转速是无刷电机转速，单位是 rpm  The speed of the C620 is directly parsed from the data feedback by the ESC, in units of rpm
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
    const float wheel_circumference_m = CHASSIS_WHEEL_DIAMETER_M * DRIVE_PI;
    const float left_mps = vx_mps - wz_radps * (CHASSIS_TRACK_WIDTH_M * 0.5f);
    const float right_mps = vx_mps + wz_radps * (CHASSIS_TRACK_WIDTH_M * 0.5f);

    // 将线速度转换成轮子转速 Convert linear speed to wheel speed
    const float left_wheels_rpm = left_mps * 60.0f / wheel_circumference_m;
    const float right_wheels_rpm = right_mps * 60.0f / wheel_circumference_m;

    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_LF, left_wheels_rpm);
    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_LB, left_wheels_rpm);
    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_RF, right_wheels_rpm);
    ChassisMotor_SetWheelTargetRpm(CHASSIS_WHEEL_RB, right_wheels_rpm);
}

// 根据遥控器输入更新底盘速度命令 Update chassis speed commands based on remote control input
void ChassisMotor_UpdateFromSbusChannels(const uint16_t channels[CHASSIS_SBUS_CH_COUNT])
{
    float vx_cmd;
    float wz_cmd;

    if (channels == NULL) {
        ChassisMotor_SetChassisSpeed(0.0f, 0.0f);
        return;
    }

    vx_cmd = ChassisMotor_NormalizeChannel(channels[CHASSIS_SBUS_VX_CH]) * CHASSIS_MAX_VX_MPS;
    wz_cmd = ChassisMotor_NormalizeChannel(channels[CHASSIS_SBUS_WZ_CH]) * CHASSIS_MAX_WZ_RADPS;

    ChassisMotor_SetChassisSpeed(vx_cmd, wz_cmd);
}

static uint8_t ChassisMotor_Stop(ChassisMotor_t *motor) {

    if(motor == NULL) {
        return 1U;
    }
    
    if ( motor->target_wheel_rpm > -1.0f && motor->target_wheel_rpm < 1.0f) {
        motor->target_wheel_rpm = 0.0f;
        motor->current_cmd = 0.0f;

        motor->speed_pid.error = 0.0f;
        motor->speed_pid.prev_error = 0.0f;
        motor->speed_pid.prev_target = 0.0f;
        motor->speed_pid.integral = 0.0f;

        return 1U;
  }
        return 0U;
}

// 底盘控制周期函数，计算每个电机的当前命令并发送到底盘控制器  Chassis control
// loop function, calculate the current command for each motor and send it to
// the chassis controllers
void ChassisMotor_ControlLoop(void) {
  for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    ChassisMotor_t *motor = &ChassisMotor_Table[i];
    motor->feedback_wheel_rpm = ChassisMotor_GetFeedbackRpm(motor->wheel);

     // 如果目标转速接近于零，则直接停止电机以避免积分累加 up  If the target speed is close to zero, stop the motor directly to avoid integral windup
    if (ChassisMotor_Stop(motor)) {
        continue;
    }

    motor->current_cmd = PID_Calculate(
        &motor->speed_pid, motor->feedback_wheel_rpm, motor->target_wheel_rpm);
    motor->current_cmd =
        ChassisMotor_Clamp(motor->current_cmd, motor->current_limit);
  }

    ChassisMotor_SendAllCurrent_Section();
}

// 发送单个电机的当前命令到对应的CAN ID  Send current command for a single motor to its corresponding CAN ID
void ChassisMotor_SendCurrent(ChassisMotor_t *motor)
{
    int16_t c620_current;
    uint8_t data[8] = {0};
    uint8_t offset;

    if (motor == NULL) {
        return;
    }

    // VESC 电机直接通过专用函数发送电流命令，C620 电机需要打包成 CAN 数据帧发送  VESC motors send current commands directly through a dedicated function, while C620 motors need to be packed into CAN data frames for sending
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

// 发送所有电机的当前命令到对应的CAN ID  Send current commands for all motors to
// their corresponding CAN IDs
void ChassisMotor_SendAllCurrent(void) {
  uint8_t can1_0x200_data[8] = {0};
  uint8_t can1_0x1ff_data[8] = {0};
  uint8_t send_can1_0x200 = 0U;
  uint8_t send_can1_0x1ff = 0U;

  // 目前只适用于 4 个电机底盘，并且3个拓展帧，一个标准帧
  for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    ChassisMotor_t *motor = &ChassisMotor_Table[i];

    if (motor->type == CHASSIS_MOTOR_TYPE_VESC) {
      ChassisMotor_SendCurrent(motor);
      continue;
    }

    int16_t current = (int16_t)ChassisMotor_Clamp(
        motor->current_cmd * motor->command_direction, motor->current_limit);
    uint8_t offset = (uint8_t)(motor->c620_tx_slot * 2U);

    // 0x200 一帧控制 ID1~ID4，0x1FF 一帧控制 ID5~ID8  0x200 controls ID1~ID4 in
    // one frame, while 0x1FF controls ID5~ID8 in one frame
    uint8_t *data =
        (motor->c620_tx_id == 0x1FFU) ? can1_0x1ff_data : can1_0x200_data;

    data[offset] = (uint8_t)(current >> 8);
    data[offset + 1U] = (uint8_t)current;

    if (motor->c620_tx_id == 0x1FFU) {
      send_can1_0x1ff = 1U;
    } else {
      send_can1_0x200 = 1U;
    }
  }

  if (send_can1_0x200 != 0U) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
      CAN_Send_Data(&hcan1, 0x200, can1_0x200_data, 8);
    }
  }
  if (send_can1_0x1ff != 0U) {
    CAN_Send_Data(&hcan1, 0x1FF, can1_0x1ff_data, 8);
  }
}

// 分时发送C620/N630电机的当前命令，避免一次发送过多数据导致 Mailbox 拥堵  Send current commands for C620 motors in a time-division manner to avoid bus congestion caused by sending too much data at once
void ChassisMotor_SendAllCurrent_Section(void) {
  uint8_t can1_0x200_data[8] = {0};
  uint8_t can1_0x1ff_data[8] = {0};
  uint8_t send_can1_0x200 = 0U;
  uint8_t send_can1_0x1ff = 0U;

  // 目前只适用于 4 个电机底盘，并且3个拓展帧，一个标准帧
for (uint32_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    ChassisMotor_t *motor = &ChassisMotor_Table[i];

     if (motor->type != CHASSIS_MOTOR_TYPE_C620) {
    continue;
    }

    int16_t current = (int16_t)ChassisMotor_Clamp(
        motor->current_cmd * motor->command_direction, motor->current_limit);
    uint8_t offset = (uint8_t)(motor->c620_tx_slot * 2U);

    // 0x200 一帧控制 ID1~ID4，0x1FF 一帧控制 ID5~ID8  0x200 controls ID1~ID4 in
    // one frame, while 0x1FF controls ID5~ID8 in one frame
    uint8_t *data =
        (motor->c620_tx_id == 0x1FFU) ? can1_0x1ff_data : can1_0x200_data;

    data[offset] = (uint8_t)(current >> 8);
    data[offset + 1U] = (uint8_t)current;

    if (motor->c620_tx_id == 0x1FFU) {
      send_can1_0x1ff = 1U;
    } else {
      send_can1_0x200 = 1U;
    }
}

    if (send_can1_0x200 != 0U) {
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
        CAN_Send_Data(&hcan1, 0x200, can1_0x200_data, 8);
      }
    }
    if (send_can1_0x1ff != 0U) {
      CAN_Send_Data(&hcan1, 0x1FF, can1_0x1ff_data, 8);
    }

    for (uint32_t n = 0U; n < CHASSIS_MOTOR_COUNT; n++) {
         static uint32_t send_index = 0U;
         static uint32_t j;
         j = (send_index + n) % CHASSIS_MOTOR_COUNT;
         ChassisMotor_t *motor = &ChassisMotor_Table[j];
         send_index = (send_index + 1U) % CHASSIS_MOTOR_COUNT;
         if (motor->type == CHASSIS_MOTOR_TYPE_VESC) {
                ChassisMotor_SendCurrent(motor);
            }
             break;
    }

  
}

