#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_app_init(void);
void motor_app_motor_task(void *argument);
void motor_app_comm_task(void *argument);
void motor_app_on_control_tick(void);

#ifdef __cplusplus
}
#endif

#endif
