#include "as5600.h"

#include "soft_i2c.h"

#define AS5600_TWO_PI  (6.28318530718f)

static as5600_config_t as5600_state = {
  7U,
  0.0f,
  1
};

static float wrap_angle(float angle)
{
  while (angle >= AS5600_TWO_PI) {
    angle -= AS5600_TWO_PI;
  }
  while (angle < 0.0f) {
    angle += AS5600_TWO_PI;
  }
  return angle;
}

HAL_StatusTypeDef as5600_init(const as5600_config_t *config)
{
  uint8_t status = 0U;

  if (config != NULL) {
    as5600_state = *config;
    if (as5600_state.pole_pairs == 0U) {
      as5600_state.pole_pairs = 1U;
    }
    if (as5600_state.direction == 0) {
      as5600_state.direction = 1;
    }
  }

  return as5600_read_status(&status);
}

HAL_StatusTypeDef as5600_read_status(uint8_t *status)
{
  if (status == NULL) {
    return HAL_ERROR;
  }
  return soft_i2c_read_reg(AS5600_I2C_ADDR, AS5600_STATUS_REG, status, 1U);
}

HAL_StatusTypeDef as5600_read_raw_angle(uint16_t *raw_angle)
{
  uint8_t bytes[2];
  HAL_StatusTypeDef status;

  if (raw_angle == NULL) {
    return HAL_ERROR;
  }

  status = soft_i2c_read_reg(AS5600_I2C_ADDR, AS5600_RAW_ANGLE_REG, bytes, 2U);
  if (status != HAL_OK) {
    return status;
  }

  *raw_angle = (uint16_t)((((uint16_t)bytes[0] << 8U) | bytes[1]) & 0x0FFFU);
  return HAL_OK;
}

HAL_StatusTypeDef as5600_read_mech_angle(float *angle_rad)
{
  uint16_t raw;
  HAL_StatusTypeDef status;
  float angle;

  if (angle_rad == NULL) {
    return HAL_ERROR;
  }

  status = as5600_read_raw_angle(&raw);
  if (status != HAL_OK) {
    return status;
  }

  angle = ((float)raw * AS5600_TWO_PI) / AS5600_RESOLUTION;
  if (as5600_state.direction < 0) {
    angle = AS5600_TWO_PI - angle;
  }
  angle -= as5600_state.zero_offset_rad;
  *angle_rad = wrap_angle(angle);
  return HAL_OK;
}

HAL_StatusTypeDef as5600_read_elec_angle(float *angle_rad)
{
  float mech_angle;
  HAL_StatusTypeDef status;

  if (angle_rad == NULL) {
    return HAL_ERROR;
  }

  status = as5600_read_mech_angle(&mech_angle);
  if (status != HAL_OK) {
    return status;
  }

  *angle_rad = wrap_angle(mech_angle * (float)as5600_state.pole_pairs);
  return HAL_OK;
}
