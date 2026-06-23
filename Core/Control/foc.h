#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "foc_math.h"

#define FOC_DEFAULT_POLE_PAIRS       (7U)
#define FOC_CONTROL_DT_SEC           (0.00005f)
#define FOC_DEFAULT_VOLTAGE_LIMIT    (6.0f)

typedef enum {
  FOC_MODE_DISABLED = 0,
  FOC_MODE_OPEN_LOOP,
  FOC_MODE_CURRENT
} foc_mode_t;

typedef struct {
  uint8_t enabled;
  uint8_t sensor_valid;
  foc_mode_t mode;
  float electrical_angle;
  float mechanical_angle;
  float id;
  float iq;
  float iq_target;
  float speed_target;
  float vbus;
  float temperature;
  foc_pwm_duty_t duty;
} foc_status_t;

void foc_init(void);
HAL_StatusTypeDef foc_enable(void);
void foc_disable(void);
void foc_fast_loop(void);
void foc_update_sensor_angle(float mechanical_angle_rad);
void foc_set_iq_target(float iq_target);
void foc_set_speed_target(float speed_target_rad_s);
void foc_set_open_loop(float iq_target, float electrical_speed_rad_s);
void foc_set_pole_pairs(uint8_t pole_pairs);
void foc_get_status(foc_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
