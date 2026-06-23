#include "pid.h"

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max,
              float integrator_min,
              float integrator_max)
{
  if (pid == 0) {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integrator = 0.0f;
  pid->previous_error = 0.0f;
  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->integrator_min = integrator_min;
  pid->integrator_max = integrator_max;
}

void pid_reset(pid_controller_t *pid)
{
  if (pid == 0) {
    return;
  }

  pid->integrator = 0.0f;
  pid->previous_error = 0.0f;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
  float error;
  float derivative;
  float output;

  if ((pid == 0) || (dt <= 0.0f)) {
    return 0.0f;
  }

  error = setpoint - measurement;
  pid->integrator += error * pid->ki * dt;
  pid->integrator = clampf(pid->integrator, pid->integrator_min, pid->integrator_max);

  derivative = (error - pid->previous_error) / dt;
  output = (pid->kp * error) + pid->integrator + (pid->kd * derivative);
  output = clampf(output, pid->output_min, pid->output_max);

  pid->previous_error = error;
  return output;
}
