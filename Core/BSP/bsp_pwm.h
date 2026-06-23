#ifndef BSP_PWM_H
#define BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_PWM_FREQUENCY_HZ        (20000U)
#define BSP_PWM_TIMER_CLOCK_HZ      (170000000U)
#define BSP_PWM_TIMER_PERIOD        (4249U)
#define BSP_PWM_DEFAULT_DEADTIME    (85U)

void bsp_pwm_init(void);
HAL_StatusTypeDef bsp_pwm_start(void);
void bsp_pwm_stop(void);
void bsp_pwm_disable_outputs(void);
void bsp_pwm_set_duty_abc(float duty_a, float duty_b, float duty_c);
uint8_t bsp_pwm_is_running(void);

#ifdef __cplusplus
}
#endif

#endif
