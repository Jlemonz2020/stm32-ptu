#ifndef FOC_MATH_H
#define FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float a;
  float b;
  float c;
} foc_abc_t;

typedef struct {
  float alpha;
  float beta;
} foc_alpha_beta_t;

typedef struct {
  float d;
  float q;
} foc_dq_t;

typedef struct {
  float a;
  float b;
  float c;
} foc_pwm_duty_t;

void foc_clarke(const foc_abc_t *abc, foc_alpha_beta_t *alpha_beta);
void foc_park(const foc_alpha_beta_t *alpha_beta, float sin_angle, float cos_angle, foc_dq_t *dq);
void foc_inv_park(const foc_dq_t *dq, float sin_angle, float cos_angle, foc_alpha_beta_t *alpha_beta);
void foc_svpwm(const foc_alpha_beta_t *voltage, float vbus, foc_pwm_duty_t *duty);
float foc_wrap_angle(float angle);

#ifdef __cplusplus
}
#endif

#endif
