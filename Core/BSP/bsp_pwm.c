#include "bsp_pwm.h"

#include "tim.h"

static uint8_t pwm_running;

static float clamp01(float value)
{
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

static uint32_t duty_to_compare(float duty)
{
  float period = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
  float compare = clamp01(duty) * period;

  if (compare < 0.0f) {
    compare = 0.0f;
  }
  if (compare > period) {
    compare = period;
  }

  return (uint32_t)(compare + 0.5f);
}

void bsp_pwm_init(void)
{
  pwm_running = 0U;
  bsp_pwm_set_duty_abc(0.5f, 0.5f, 0.5f);
  bsp_pwm_disable_outputs();
}

HAL_StatusTypeDef bsp_pwm_start(void)
{
  HAL_StatusTypeDef status;

  bsp_pwm_set_duty_abc(0.5f, 0.5f, 0.5f);

  status = HAL_TIM_Base_Start_IT(&htim1);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  if (status != HAL_OK) {
    return status;
  }
  status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  if (status != HAL_OK) {
    return status;
  }
  status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  if (status != HAL_OK) {
    return status;
  }
  status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  if (status != HAL_OK) {
    return status;
  }

  __HAL_TIM_MOE_ENABLE(&htim1);
  pwm_running = 1U;
  return HAL_OK;
}

void bsp_pwm_stop(void)
{
  bsp_pwm_disable_outputs();

  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_Base_Stop_IT(&htim1);

  bsp_pwm_set_duty_abc(0.5f, 0.5f, 0.5f);
  pwm_running = 0U;
}

void bsp_pwm_disable_outputs(void)
{
  __HAL_TIM_MOE_DISABLE(&htim1);
  pwm_running = 0U;
}

void bsp_pwm_set_duty_abc(float duty_a, float duty_b, float duty_c)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_to_compare(duty_a));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_to_compare(duty_b));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_to_compare(duty_c));
}

uint8_t bsp_pwm_is_running(void)
{
  return pwm_running;
}
