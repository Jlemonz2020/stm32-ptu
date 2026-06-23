#include "soft_i2c.h"

#define SOFT_I2C_TIMEOUT_CYCLES  (2000U)

static void delay_us(uint32_t us)
{
  uint32_t cycles;
  uint32_t start;

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) {
    cycles = (SystemCoreClock / 1000000U) * us;
    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
      __NOP();
    }
  } else {
    cycles = (SystemCoreClock / 1000000U) * us / 6U;
    while (cycles-- > 0U) {
      __NOP();
    }
  }
}

static void scl_high(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
}

static void scl_low(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
}

static void sda_high(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET);
}

static void sda_low(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET);
}

static GPIO_PinState sda_read(void)
{
  return HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN);
}

static HAL_StatusTypeDef wait_scl_high(void)
{
  uint32_t timeout = SOFT_I2C_TIMEOUT_CYCLES;

  scl_high();
  while ((HAL_GPIO_ReadPin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN) == GPIO_PIN_RESET) &&
         (timeout > 0U)) {
    timeout--;
  }

  return (timeout == 0U) ? HAL_TIMEOUT : HAL_OK;
}

static HAL_StatusTypeDef clock_high_delay(void)
{
  HAL_StatusTypeDef status = wait_scl_high();
  if (status != HAL_OK) {
    return status;
  }
  delay_us(SOFT_I2C_DELAY_US);
  return HAL_OK;
}

static HAL_StatusTypeDef i2c_start(void)
{
  sda_high();
  if (clock_high_delay() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  sda_low();
  delay_us(SOFT_I2C_DELAY_US);
  scl_low();
  delay_us(SOFT_I2C_DELAY_US);
  return HAL_OK;
}

static void i2c_stop(void)
{
  sda_low();
  delay_us(SOFT_I2C_DELAY_US);
  scl_high();
  delay_us(SOFT_I2C_DELAY_US);
  sda_high();
  delay_us(SOFT_I2C_DELAY_US);
}

static HAL_StatusTypeDef write_byte(uint8_t byte)
{
  uint8_t mask;
  GPIO_PinState ack;

  for (mask = 0x80U; mask != 0U; mask >>= 1U) {
    if ((byte & mask) != 0U) {
      sda_high();
    } else {
      sda_low();
    }
    delay_us(SOFT_I2C_DELAY_US);
    if (clock_high_delay() != HAL_OK) {
      return HAL_TIMEOUT;
    }
    scl_low();
    delay_us(SOFT_I2C_DELAY_US);
  }

  sda_high();
  delay_us(SOFT_I2C_DELAY_US);
  if (clock_high_delay() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  ack = sda_read();
  scl_low();
  delay_us(SOFT_I2C_DELAY_US);

  return (ack == GPIO_PIN_RESET) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef read_byte(uint8_t *byte, uint8_t ack)
{
  uint8_t i;
  uint8_t value = 0U;

  if (byte == NULL) {
    return HAL_ERROR;
  }

  sda_high();
  for (i = 0U; i < 8U; i++) {
    value <<= 1U;
    if (clock_high_delay() != HAL_OK) {
      return HAL_TIMEOUT;
    }
    if (sda_read() == GPIO_PIN_SET) {
      value |= 1U;
    }
    scl_low();
    delay_us(SOFT_I2C_DELAY_US);
  }

  if (ack != 0U) {
    sda_low();
  } else {
    sda_high();
  }
  delay_us(SOFT_I2C_DELAY_US);
  if (clock_high_delay() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  scl_low();
  sda_high();
  delay_us(SOFT_I2C_DELAY_US);

  *byte = value;
  return HAL_OK;
}

void soft_i2c_init(void)
{
  GPIO_InitTypeDef gpio;
  uint8_t i;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  sda_high();
  scl_high();

  for (i = 0U; i < 9U; i++) {
    scl_low();
    delay_us(SOFT_I2C_DELAY_US);
    scl_high();
    delay_us(SOFT_I2C_DELAY_US);
  }
  i2c_stop();
}

HAL_StatusTypeDef soft_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint16_t i;
  HAL_StatusTypeDef status;

  if ((data == NULL) || (len == 0U)) {
    return HAL_ERROR;
  }

  status = i2c_start();
  if (status != HAL_OK) {
    i2c_stop();
    return status;
  }
  if (write_byte((uint8_t)(dev_addr << 1U)) != HAL_OK) {
    i2c_stop();
    return HAL_ERROR;
  }
  if (write_byte(reg) != HAL_OK) {
    i2c_stop();
    return HAL_ERROR;
  }
  if (i2c_start() != HAL_OK) {
    i2c_stop();
    return HAL_TIMEOUT;
  }
  if (write_byte((uint8_t)((dev_addr << 1U) | 1U)) != HAL_OK) {
    i2c_stop();
    return HAL_ERROR;
  }

  for (i = 0U; i < len; i++) {
    status = read_byte(&data[i], (uint8_t)(i < (uint16_t)(len - 1U)));
    if (status != HAL_OK) {
      i2c_stop();
      return status;
    }
  }

  i2c_stop();
  return HAL_OK;
}

HAL_StatusTypeDef soft_i2c_write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
  uint16_t i;
  HAL_StatusTypeDef status;

  if ((data == NULL) && (len > 0U)) {
    return HAL_ERROR;
  }

  status = i2c_start();
  if (status != HAL_OK) {
    i2c_stop();
    return status;
  }
  if (write_byte((uint8_t)(dev_addr << 1U)) != HAL_OK) {
    i2c_stop();
    return HAL_ERROR;
  }
  if (write_byte(reg) != HAL_OK) {
    i2c_stop();
    return HAL_ERROR;
  }

  for (i = 0U; i < len; i++) {
    if (write_byte(data[i]) != HAL_OK) {
      i2c_stop();
      return HAL_ERROR;
    }
  }

  i2c_stop();
  return HAL_OK;
}
