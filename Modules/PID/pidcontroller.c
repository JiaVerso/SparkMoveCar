#include "pidcontroller.h"

/**
  * @brief  初始化参数
  * @param  hcan can接口
  * @param  Object_Para  
  * @retval None
  */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_integral, float max_output) {

    // 初始化参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_integral = max_integral;
    pid->max_output = max_output;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

/**
  * @brief  积分限幅
  * @param  hcan can接口
  * @param  Object_Para  
  * @retval None
  */
static float Limit_Value(float value, float max_value) {
    if (value >= max_value) {
        return max_value;
    } else if (value <= -max_value) {
        return -max_value;
    }
    return value;
}

/**
  * @brief  计算pid函数
  * @param  hcan can接口
  * @param  Object_Para  
  * @retval None
  */
float PID_Calculate(PID_t *pid, float input, float setpoint) {

    // 计算误差
    pid->prev_error = pid->error;
    pid->error = setpoint - input;

    // 分别计算 P、I、D 三项
    float p_out = pid->error * pid->Kp;
    float d_out = (pid->error - pid->prev_error) * pid->Kd;

    // 积分累加与积分限幅
    pid->integral += (pid->error * pid->Ki);
    pid->integral = Limit_Value(pid->integral, pid->max_integral);

    // 计算输出
    float output = p_out + pid->integral + d_out;

    // 输出限幅
    output = Limit_Value(output, pid->max_output);

    return output;
}
