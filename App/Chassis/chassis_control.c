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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "chassis.h"
#include "uxrce_app.h"
#include "bsp_sbus.h"
#include "chassis_control.h"
#include "uxrce_sub_ackermann.h"

#define CHASSIS_AUTO_TIMEOUT_MS 30000U

static uint32_t tick;
static uint16_t channels[CHASSIS_SBUS_CH_COUNT] = {0};
static AckermannDriveCmd out;


// 模式判断
static uint16_t ChassisControl_GetModeFromSbus(void)
{
    if (!SBUS_GetChannels(channels, CHASSIS_SBUS_CH_COUNT)) {
        return CHASSIS_CTRL_FASTSTOP;
    }

    if (channels[CHASSIS_MODE_CHECK_SW_CH] > 1500) {
        return CHASSIS_CTRL_MANUAL;
    } else {
        return CHASSIS_CTRL_AUTO;
    }
}

// 遥控模式--SBUS转换函数
static bool ChassisControl_GetManualCmd(ChassisAckermannCmd *cmd)
{
    if (cmd == NULL) {
        return false;
    }

    float vx_cmd = ChassisMotor_NormalizeChannel(channels[CHASSIS_SBUS_VX_CH]) * CHASSIS_MAX_VX_MPS;
    float steer_rad = ChassisMotor_NormalizeSteerChannel(channels[CHASSIS_SBUS_DM_RADPS_CH]) * CHASSIS_MAX_STEER_RAD;

    cmd->vx_mps = vx_cmd;
    cmd->steer_rad = steer_rad;
    cmd->valid = true;

    return true;
}

// 自主模式--NAV2转换函数
static bool ChassisControl_GetAutoCmd(ChassisAckermannCmd *cmd)
{
    if (cmd == NULL) {
        return false;
    }

    if (!Uxrce_SubAckermann_GetLatest(&out, &tick)) {
        return false;
    }

    if (HAL_GetTick() - tick > CHASSIS_AUTO_TIMEOUT_MS) {
        return false;
    }
    cmd->vx_mps = out.speed;
    cmd->steer_rad = out.steering_angle;
    cmd->valid = true;

    int32_t cmd_speed = (int32_t)(cmd->vx_mps * 100.0f);
    int32_t cmd_steer = (int32_t)(cmd->steer_rad * 100.0f);

    printf("auto cmd speed=%ld steer=%ld\r\n", (long)cmd_speed, (long)cmd_steer);
    return true;
}

void ChassisControl_Update(void)
{
    ChassisAckermannCmd cmd = {0};

    ChassisCtrlMode mode = ChassisControl_GetModeFromSbus();

    // if (SBUS_IsFailsafe()) {
    //     ChassisMotor_SetAckermann(0.0f, 0.0f);
    //     return;
    // }

    switch (mode) {
    case CHASSIS_CTRL_MANUAL:
        if (ChassisControl_GetManualCmd(&cmd)) {
            ChassisMotor_SetAckermann(cmd.vx_mps, cmd.steer_rad);
        } else {
            ChassisMotor_SetAckermann(0.0f, 0.0f);
        }
        break;

    case CHASSIS_CTRL_AUTO:
        if (ChassisControl_GetAutoCmd(&cmd)) {
            ChassisMotor_SetAckermann(cmd.vx_mps, cmd.steer_rad);
        } else {
            ChassisMotor_SetAckermann(0.0f, 0.0f);
        }
        break;

    default:
        ChassisMotor_SetAckermann(0.0f, 0.0f);
        break;
    }
}