/**
 * @file chassis.c
 * @author JiaVerso 
 * @brief 
 * @version 0.1
 * @date 2026-6-9
 *
 * @copyright Copyright (c) 2026 JiaVerso
 *
 */
#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_CTRL_FASTSTOP = 0,
    CHASSIS_CTRL_MANUAL,
    CHASSIS_CTRL_AUTO,
} ChassisCtrlMode;

typedef struct {
    float vx_mps;
    float steer_rad;
    bool valid;
} ChassisAckermannCmd;

void ChassisControl_Update(void);

#ifdef __cplusplus
}
#endif

#endif



