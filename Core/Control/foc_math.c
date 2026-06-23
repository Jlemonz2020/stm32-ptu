#include "foc_math.h"

#define FOC_SQRT3      (1.73205080757f)
#define FOC_SQRT3_BY_2 (0.86602540378f)
#define FOC_TWO_PI     (6.28318530718f)

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

static float max3(float a, float b, float c)
{
  float max_value = a;
  if (b > max_value) {
    max_value = b;
  }
  if (c > max_value) {
    max_value = c;
  }
  return max_value;
}

static float min3(float a, float b, float c)
{
  float min_value = a;
  if (b < min_value) {
    min_value = b;
  }
  if (c < min_value) {
    min_value = c;
  }
  return min_value;
}

void foc_clarke(const foc_abc_t *abc, foc_alpha_beta_t *alpha_beta)
{
  if ((abc == 0) || (alpha_beta == 0)) {
    return;
  }

  alpha_beta->alpha = abc->a;
  alpha_beta->beta = (abc->a + (2.0f * abc->b)) / FOC_SQRT3;
}

void foc_park(const foc_alpha_beta_t *alpha_beta, float sin_angle, float cos_angle, foc_dq_t *dq)
{
  if ((alpha_beta == 0) || (dq == 0)) {
    return;
  }

  dq->d = (alpha_beta->alpha * cos_angle) + (alpha_beta->beta * sin_angle);
  dq->q = (-alpha_beta->alpha * sin_angle) + (alpha_beta->beta * cos_angle);
}

void foc_inv_park(const foc_dq_t *dq, float sin_angle, float cos_angle, foc_alpha_beta_t *alpha_beta)
{
  if ((dq == 0) || (alpha_beta == 0)) {
    return;
  }

  alpha_beta->alpha = (dq->d * cos_angle) - (dq->q * sin_angle);
  alpha_beta->beta = (dq->d * sin_angle) + (dq->q * cos_angle);
}

void foc_svpwm(const foc_alpha_beta_t *voltage, float vbus, foc_pwm_duty_t *duty)
{
  float va;
  float vb;
  float vc;
  float vmax;
  float vmin;
  float v_offset;

  if ((voltage == 0) || (duty == 0)) {
    return;
  }

  if (vbus < 1.0f) {
    duty->a = 0.5f;
    duty->b = 0.5f;
    duty->c = 0.5f;
    return;
  }

  va = voltage->alpha;
  vb = (-0.5f * voltage->alpha) + (FOC_SQRT3_BY_2 * voltage->beta);
  vc = (-0.5f * voltage->alpha) - (FOC_SQRT3_BY_2 * voltage->beta);

  vmax = max3(va, vb, vc);
  vmin = min3(va, vb, vc);
  v_offset = -0.5f * (vmax + vmin);

  duty->a = clampf(0.5f + ((va + v_offset) / vbus), 0.0f, 1.0f);
  duty->b = clampf(0.5f + ((vb + v_offset) / vbus), 0.0f, 1.0f);
  duty->c = clampf(0.5f + ((vc + v_offset) / vbus), 0.0f, 1.0f);
}

float foc_wrap_angle(float angle)
{
  while (angle >= FOC_TWO_PI) {
    angle -= FOC_TWO_PI;
  }
  while (angle < 0.0f) {
    angle += FOC_TWO_PI;
  }
  return angle;
}
