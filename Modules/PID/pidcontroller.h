#include <iostream>

#include <cstdio>

using namespace std;

class PIDController {
public:
  /* Constructor */

  PIDController();

  /* Destructor */

  virtual ~PIDController();

  /*
   * Method
   * @function : Initialize pid parameters
   */
  void PIDCon_Init(float Kp, float Ki, float Kd, float max_integral,
                   float max_output);

  /*
   * Calculation
   * @function : Calculate pid output
   */
  float PIDCon_Calculate(float input, float setpoint);

  float Limit_integral(float integral);

  float Limit_output(float output);

  /*
   * Coefficient
   * @param Kp: Proportional gain
   * @param Ki: Integral gain
   * @param Kd: Derivative gain
   */
  float Kp;
  float Ki;
  float Kd;

  float input, output, setpoint;
  float error, prev_error;
  float integral, max_integral, max_output;

private:
};