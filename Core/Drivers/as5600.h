#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define AS5600_I2C_ADDR        (0x36U)
#define AS5600_RAW_ANGLE_REG   (0x0CU)
#define AS5600_ANGLE_REG       (0x0EU)
#define AS5600_STATUS_REG      (0x0BU)
#define AS5600_RESOLUTION      (4096.0f)

typedef struct {
  uint8_t pole_pairs;
  float zero_offset_rad;
  int8_t direction;
} as5600_config_t;

HAL_StatusTypeDef as5600_init(const as5600_config_t *config);
HAL_StatusTypeDef as5600_read_raw_angle(uint16_t *raw_angle);
HAL_StatusTypeDef as5600_read_mech_angle(float *angle_rad);
HAL_StatusTypeDef as5600_read_elec_angle(float *angle_rad);
HAL_StatusTypeDef as5600_read_status(uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif
