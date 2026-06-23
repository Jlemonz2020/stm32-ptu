#include "foc.h"

#include <math.h>
#include <string.h>

#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "pid.h"

#define FOC_TWO_PI             (6.28318530718f)
#define FOC_CURRENT_KP         (0.35f)
#define FOC_CURRENT_KI         (80.0f)
#define FOC_CURRENT_KD         (0.0f)
#define FOC_OPEN_LOOP_SPEED    (12.0f)

typedef struct {
  foc_status_t status;
  pid_controller_t id_pid;
  pid_controller_t iq_pid;
  float open_loop_angle;
  float open_loop_speed;
  uint8_t pole_pairs;
  float voltage_limit;
} foc_state_t;

static foc_state_t foc_state;

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

void foc_init(void)
{
  (void)memset(&foc_state, 0, sizeof(foc_state));
  foc_state.pole_pairs = FOC_DEFAULT_POLE_PAIRS;
  foc_state.voltage_limit = FOC_DEFAULT_VOLTAGE_LIMIT;
  foc_state.open_loop_speed = FOC_OPEN_LOOP_SPEED;
  foc_state.status.mode = FOC_MODE_DISABLED;
  foc_state.status.duty.a = 0.5f;
  foc_state.status.duty.b = 0.5f;
  foc_state.status.duty.c = 0.5f;

  pid_init(&foc_state.id_pid,
           FOC_CURRENT_KP,
           FOC_CURRENT_KI,
           FOC_CURRENT_KD,
           -FOC_DEFAULT_VOLTAGE_LIMIT,
           FOC_DEFAULT_VOLTAGE_LIMIT,
           -FOC_DEFAULT_VOLTAGE_LIMIT,
           FOC_DEFAULT_VOLTAGE_LIMIT);

  pid_init(&foc_state.iq_pid,
           FOC_CURRENT_KP,
           FOC_CURRENT_KI,
           FOC_CURRENT_KD,
           -FOC_DEFAULT_VOLTAGE_LIMIT,
           FOC_DEFAULT_VOLTAGE_LIMIT,
           -FOC_DEFAULT_VOLTAGE_LIMIT,
           FOC_DEFAULT_VOLTAGE_LIMIT);
}

HAL_StatusTypeDef foc_enable(void)
{
  HAL_StatusTypeDef status;

  pid_reset(&foc_state.id_pid);
  pid_reset(&foc_state.iq_pid);
  foc_state.open_loop_angle = foc_state.status.electrical_angle;
  foc_state.status.mode = foc_state.status.sensor_valid ? FOC_MODE_CURRENT : FOC_MODE_OPEN_LOOP;

  status = bsp_pwm_start();
  if (status == HAL_OK) {
    foc_state.status.enabled = 1U;
  }
  return status;
}

void foc_disable(void)
{
  foc_state.status.enabled = 0U;
  foc_state.status.mode = FOC_MODE_DISABLED;
  foc_state.status.duty.a = 0.5f;
  foc_state.status.duty.b = 0.5f;
  foc_state.status.duty.c = 0.5f;
  bsp_pwm_stop();
}

void foc_update_sensor_angle(float mechanical_angle_rad)
{
  foc_state.status.mechanical_angle = foc_wrap_angle(mechanical_angle_rad);
  foc_state.status.electrical_angle = foc_wrap_angle(mechanical_angle_rad * (float)foc_state.pole_pairs);
  foc_state.status.sensor_valid = 1U;
}

void foc_set_iq_target(float iq_target)
{
  foc_state.status.iq_target = clampf(iq_target, -20.0f, 20.0f);
  if (foc_state.status.enabled != 0U) {
    foc_state.status.mode = foc_state.status.sensor_valid ? FOC_MODE_CURRENT : FOC_MODE_OPEN_LOOP;
  }
}

void foc_set_speed_target(float speed_target_rad_s)
{
  foc_state.status.speed_target = speed_target_rad_s;
  foc_state.open_loop_speed = speed_target_rad_s * (float)foc_state.pole_pairs;
}

void foc_set_open_loop(float iq_target, float electrical_speed_rad_s)
{
  foc_set_iq_target(iq_target);
  foc_state.open_loop_speed = electrical_speed_rad_s;
  if (foc_state.status.enabled != 0U) {
    foc_state.status.mode = FOC_MODE_OPEN_LOOP;
  }
}

void foc_set_pole_pairs(uint8_t pole_pairs)
{
  if (pole_pairs == 0U) {
    pole_pairs = 1U;
  }
  foc_state.pole_pairs = pole_pairs;
}

void foc_fast_loop(void)
{
  bsp_phase_currents_t currents;
  foc_abc_t abc;
  foc_alpha_beta_t current_ab;
  foc_alpha_beta_t voltage_ab;
  foc_dq_t current_dq;
  foc_dq_t voltage_dq;
  foc_pwm_duty_t duty;
  float angle;
  float sin_angle;
  float cos_angle;
  float vbus;

  if (foc_state.status.enabled == 0U) {
    return;
  }

  if (bsp_adc_get_phase_currents(&currents) != HAL_OK) {
    bsp_pwm_disable_outputs();
    foc_state.status.enabled = 0U;
    foc_state.status.mode = FOC_MODE_DISABLED;
    return;
  }

  vbus = bsp_adc_get_vbus();
  foc_state.status.vbus = vbus;

  if ((foc_state.status.mode == FOC_MODE_OPEN_LOOP) || (foc_state.status.sensor_valid == 0U)) {
    foc_state.open_loop_angle = foc_wrap_angle(foc_state.open_loop_angle +
                                               (foc_state.open_loop_speed * FOC_CONTROL_DT_SEC));
    angle = foc_state.open_loop_angle;
  } else {
    angle = foc_state.status.electrical_angle;
  }

  sin_angle = sinf(angle);
  cos_angle = cosf(angle);

  abc.a = currents.ia;
  abc.b = currents.ib;
  abc.c = currents.ic;
  foc_clarke(&abc, &current_ab);
  foc_park(&current_ab, sin_angle, cos_angle, &current_dq);

  voltage_dq.d = pid_update(&foc_state.id_pid, 0.0f, current_dq.d, FOC_CONTROL_DT_SEC);
  voltage_dq.q = pid_update(&foc_state.iq_pid,
                            foc_state.status.iq_target,
                            current_dq.q,
                            FOC_CONTROL_DT_SEC);
  voltage_dq.d = clampf(voltage_dq.d, -foc_state.voltage_limit, foc_state.voltage_limit);
  voltage_dq.q = clampf(voltage_dq.q, -foc_state.voltage_limit, foc_state.voltage_limit);

  foc_inv_park(&voltage_dq, sin_angle, cos_angle, &voltage_ab);
  foc_svpwm(&voltage_ab, vbus, &duty);
  bsp_pwm_set_duty_abc(duty.a, duty.b, duty.c);

  foc_state.status.electrical_angle = angle;
  foc_state.status.id = current_dq.d;
  foc_state.status.iq = current_dq.q;
  foc_state.status.duty = duty;
}

void foc_get_status(foc_status_t *status)
{
  if (status == 0) {
    return;
  }
  *status = foc_state.status;
  status->temperature = bsp_adc_get_temp();
}
