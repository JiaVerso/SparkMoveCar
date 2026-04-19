#include "PIDController.h"
#include <iostream>

using namespace std;

// 全局变量存储PID参数
float Kp, Ki, Kd;
float error, prev_error;
float integral, max_integral, max_output;

void PIDCon_Init(float _Kp, float _Ki, float _Kd, float _max_integral,
                 float _max_output) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  max_integral = _max_integral;
  max_output = _max_output;
  error = 0;
  prev_error = 0;
  integral = 0;
}
/*
 *@param: 积分限幅
 */
float Limit_integral(float integral_val) {
  if (integral_val >= max_integral) {
    return max_integral;
  } else if (integral_val <= -max_integral) {
    return -max_integral;
  }
  return integral_val;
}
/*
 *@param: 输出限幅
 */
float Limit_output(float output_val) {
  if (output_val >= max_output) {
    return max_output;
  } else if (output_val <= -max_output) {
    return -max_output;
  }
  return output_val;
}

float PIDCon_Calculate(float input, float setpoint) {
  prev_error = error;
  error = setpoint - input;

  float d_out = (error - prev_error) * Kd;
  float p_out = error * Kp;
  integral += error * Ki;

  integral = Limit_integral(integral);

  cout << "d_out: " << d_out << ", p_out: " << p_out
       << ", integral: " << integral << endl;

  float output = d_out + p_out + integral;

  output = Limit_output(output);

  cout << "output: " << output << endl;

  return output;
}
