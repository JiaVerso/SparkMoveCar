/**
 * @file app_test.h
 * @author JiaVerso
 * @brief 应用测试层
 * @version 0.1
 * @date 2026-04-015
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include <stdint.h>

#define PI 3.1415926

/**
 * @brief PID 控制器结构体
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;

    float max_integral;
    float max_output;
    
    float error;
    float prev_error;
    float prev_target;
    float integral;
} PID_t;

/**
 * @brief  初始化 PID 控制器参数
 * @param  pid          目标 PID 对象指针
 * @param  kp           比例系数
 * @param  ki           积分系数
 * @param  kd           微分系数
 * @param  kf           前馈系数
 * @param  max_integral 积分限幅最大值
 * @param  max_output   输出限幅最大值
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf, float max_integral, float max_output);

/**
 * @brief  计算 PID 输出
 * @param  pid          目标 PID 对象指针
 * @param  input        当前实际测量值 (比如当前真实转速)
 * @param  setpoint     目标期望值 (比如目标转速)
 * @retval float        PID 计算后的控制量输出
 */
float PID_Calculate(PID_t *pid, float input, float setpoint);

#endif /* __PIDCONTROLLER_H */