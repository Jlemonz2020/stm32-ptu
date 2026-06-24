#include "drv8316.h"

#include "spi.h"

#define DRV8316_SPI_TIMEOUT_MS       (2U)
#define DRV8316_ADDR_MASK            (0x3FU)
#define DRV8316_SPI_STATUS_RESERVED  (0x80U)

#define DRV8316_CTRL1_UNLOCK         (0x03U)
#define DRV8316_CTRL2_RESERVED       (0xC0U)
#define DRV8316_CTRL2_SDO_PUSH_PULL  (1U << 5)
#define DRV8316_CTRL2_CLR_FLT        (1U << 0)
#define DRV8316_CTRL3_DEFAULT        (0x46U)
#define DRV8316_CTRL4_OCP_DEG_SHIFT  (4U)
#define DRV8316_CTRL4_OCP_RETRY      (1U << 3)
#define DRV8316_CTRL4_OCP_LVL        (1U << 2)
#define DRV8316_CTRL5_EN_AAR         (1U << 3)
#define DRV8316_CTRL5_EN_ASR         (1U << 2)
#define DRV8316_CTRL10_DLYCMP_EN     (1U << 4)

static HAL_StatusTypeDef transfer(uint16_t tx, uint16_t *rx);

static uint8_t is_valid_address(uint8_t address)
{
  return (address <= DRV8316_REG_CTRL_6) || (address == DRV8316_REG_CTRL_10);
}

static uint8_t popcount16(uint16_t value)
{
  uint8_t count = 0U;

  while (value != 0U) {
    count = (uint8_t)(count + (value & 1U));
    value >>= 1U;
  }

  return count;
}

static uint16_t make_frame(uint8_t read, uint8_t address, uint8_t data)
{
  uint16_t frame = 0U;

  if (read != 0U) {
    frame |= (1U << 15);
  }
  frame |= ((uint16_t)(address & DRV8316_ADDR_MASK) << 9);
  frame |= data;

  if ((popcount16(frame) & 1U) != 0U) {
    frame |= (1U << 8);
  }

  return frame;
}

static void cs_low(void)
{
  HAL_GPIO_WritePin(DRV8316_NCS_GPIO_Port, DRV8316_NCS_Pin, GPIO_PIN_RESET);
}

static void cs_high(void)
{
  HAL_GPIO_WritePin(DRV8316_NCS_GPIO_Port, DRV8316_NCS_Pin, GPIO_PIN_SET);
}

static void ncs_gap(void)
{
  volatile uint32_t cycles;

  for (cycles = 0U; cycles < 64U; cycles++) {
    __NOP();
  }
}

static HAL_StatusTypeDef transfer(uint16_t tx, uint16_t *rx)
{
  HAL_StatusTypeDef status;

  if (rx == 0) {
    return HAL_ERROR;
  }

  cs_low();
  status = HAL_SPI_TransmitReceive(&hspi1,
                                   (uint8_t *)&tx,
                                   (uint8_t *)rx,
                                   1U,
                                   DRV8316_SPI_TIMEOUT_MS);
  cs_high();
  ncs_gap();

  if (status != HAL_OK) {
    return status;
  }

  if (((*rx >> 8) & DRV8316_SPI_STATUS_RESERVED) != 0U) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void drv8316_get_default_config(drv8316_config_t *config)
{
  if (config == 0) {
    return;
  }

  config->pwm_mode = DRV8316_PWM_MODE_6X;
  config->slew = DRV8316_SLEW_25V_US;
  config->csa_gain = DRV8316_CSA_GAIN_0P15_V_PER_A;
  config->ocp_deglitch = DRV8316_OCP_DEG_0P6_US;
  config->ocp_mode = DRV8316_OCP_MODE_LATCHED;
  config->ocp_retry_500ms = 0U;
  config->ocp_level_24a = 0U;
  config->enable_active_rectification = 0U;
  config->enable_delay_compensation = 0U;
  config->delay_target = 0U;
}

HAL_StatusTypeDef drv8316_read_register(uint8_t address, uint8_t *data, uint8_t *spi_status)
{
  uint16_t rx = 0U;
  HAL_StatusTypeDef status;

  if ((data == 0) || (is_valid_address(address) == 0U)) {
    return HAL_ERROR;
  }

  status = transfer(make_frame(1U, address, 0U), &rx);
  if (status != HAL_OK) {
    return status;
  }

  if (spi_status != 0) {
    *spi_status = (uint8_t)(rx >> 8);
  }
  *data = (uint8_t)(rx & 0xFFU);
  return HAL_OK;
}

HAL_StatusTypeDef drv8316_write_register(uint8_t address, uint8_t data, uint8_t *spi_status)
{
  uint16_t rx = 0U;
  HAL_StatusTypeDef status;

  if (is_valid_address(address) == 0U) {
    return HAL_ERROR;
  }

  status = transfer(make_frame(0U, address, data), &rx);
  if (status != HAL_OK) {
    return status;
  }

  if (spi_status != 0) {
    *spi_status = (uint8_t)(rx >> 8);
  }
  return HAL_OK;
}

HAL_StatusTypeDef drv8316_configure(const drv8316_config_t *config)
{
  uint8_t ctrl2;
  uint8_t ctrl4;
  uint8_t ctrl5;
  uint8_t ctrl10;
  uint8_t spi_status;

  if (config == 0) {
    return HAL_ERROR;
  }

  if (drv8316_write_register(DRV8316_REG_CTRL_1, DRV8316_CTRL1_UNLOCK, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }

  ctrl2 = (uint8_t)(DRV8316_CTRL2_RESERVED |
                    DRV8316_CTRL2_SDO_PUSH_PULL |
                    (((uint8_t)config->slew & 0x03U) << 3) |
                    (((uint8_t)config->pwm_mode & 0x03U) << 1));
  if (drv8316_write_register(DRV8316_REG_CTRL_2, ctrl2, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }

  if (drv8316_write_register(DRV8316_REG_CTRL_3, DRV8316_CTRL3_DEFAULT, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }

  ctrl4 = (uint8_t)((((uint8_t)config->ocp_deglitch & 0x03U) << DRV8316_CTRL4_OCP_DEG_SHIFT) |
                    ((uint8_t)config->ocp_mode & 0x03U));
  if (config->ocp_retry_500ms != 0U) {
    ctrl4 |= DRV8316_CTRL4_OCP_RETRY;
  }
  if (config->ocp_level_24a != 0U) {
    ctrl4 |= DRV8316_CTRL4_OCP_LVL;
  }
  if (drv8316_write_register(DRV8316_REG_CTRL_4, ctrl4, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }

  ctrl5 = (uint8_t)((uint8_t)config->csa_gain & 0x03U);
  if (config->enable_active_rectification != 0U) {
    ctrl5 |= (DRV8316_CTRL5_EN_AAR | DRV8316_CTRL5_EN_ASR);
  }
  if (drv8316_write_register(DRV8316_REG_CTRL_5, ctrl5, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }

  ctrl10 = 0U;
  if (config->enable_delay_compensation != 0U) {
    ctrl10 = (uint8_t)(DRV8316_CTRL10_DLYCMP_EN | (config->delay_target & 0x0FU));
  }
  if (drv8316_write_register(DRV8316_REG_CTRL_10, ctrl10, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef drv8316_init(const drv8316_config_t *config)
{
  drv8316_config_t default_config;

  cs_high();
  HAL_Delay(2U);

  if (config == 0) {
    drv8316_get_default_config(&default_config);
    config = &default_config;
  }

  if (drv8316_configure(config) != HAL_OK) {
    return HAL_ERROR;
  }

  return drv8316_clear_faults();
}

HAL_StatusTypeDef drv8316_read_status(drv8316_status_t *status)
{
  uint8_t spi_status;

  if (status == 0) {
    return HAL_ERROR;
  }

  if (drv8316_read_register(DRV8316_REG_IC_STATUS, &status->ic_status, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }
  status->spi_status = spi_status;

  if (drv8316_read_register(DRV8316_REG_STATUS_1, &status->status1, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }
  status->spi_status |= spi_status;

  if (drv8316_read_register(DRV8316_REG_STATUS_2, &status->status2, &spi_status) != HAL_OK) {
    return HAL_ERROR;
  }
  status->spi_status |= spi_status;

  return HAL_OK;
}

HAL_StatusTypeDef drv8316_clear_faults(void)
{
  uint8_t ctrl2;

  if (drv8316_read_register(DRV8316_REG_CTRL_2, &ctrl2, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  return drv8316_write_register(DRV8316_REG_CTRL_2, (uint8_t)(ctrl2 | DRV8316_CTRL2_CLR_FLT), 0);
}

float drv8316_csa_gain_volts_per_amp(drv8316_csa_gain_t gain)
{
  switch (gain) {
    case DRV8316_CSA_GAIN_0P30_V_PER_A:
      return 0.30f;
    case DRV8316_CSA_GAIN_0P60_V_PER_A:
      return 0.60f;
    case DRV8316_CSA_GAIN_1P20_V_PER_A:
      return 1.20f;
    case DRV8316_CSA_GAIN_0P15_V_PER_A:
    default:
      return 0.15f;
  }
}

uint8_t drv8316_status_has_fault(const drv8316_status_t *status)
{
  if (status == 0) {
    return 0U;
  }

  return ((status->ic_status & (DRV8316_IC_STATUS_FAULT |
                                DRV8316_IC_STATUS_SPI_FLT |
                                DRV8316_IC_STATUS_OCP |
                                DRV8316_IC_STATUS_OVP |
                                DRV8316_IC_STATUS_OT |
                                DRV8316_IC_STATUS_BK_FLT)) != 0U) ||
         (status->status1 != 0U) ||
         (status->status2 != 0U);
}
