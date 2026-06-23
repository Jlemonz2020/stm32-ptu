#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp;
  float ki;
  float kd;
  float integrator;
  float previous_error;
  float output_min;
  float output_max;
  float integrator_min;
  float integrator_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max,
              float integrator_min,
              float integrator_max);
void pid_reset(pid_controller_t *pid);
float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif
