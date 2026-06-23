#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SOFT_I2C_SCL_PORT    GPIOB
#define SOFT_I2C_SCL_PIN     GPIO_PIN_6
#define SOFT_I2C_SDA_PORT    GPIOB
#define SOFT_I2C_SDA_PIN     GPIO_PIN_7
#define SOFT_I2C_DELAY_US    (3U)

void soft_i2c_init(void);
HAL_StatusTypeDef soft_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef soft_i2c_write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
