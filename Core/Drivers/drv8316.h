#ifndef DRV8316_H
#define DRV8316_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DRV8316_REG_IC_STATUS   (0x00U)
#define DRV8316_REG_STATUS_1    (0x01U)
#define DRV8316_REG_STATUS_2    (0x02U)
#define DRV8316_REG_CTRL_1      (0x03U)
#define DRV8316_REG_CTRL_2      (0x04U)
#define DRV8316_REG_CTRL_3      (0x05U)
#define DRV8316_REG_CTRL_4      (0x06U)
#define DRV8316_REG_CTRL_5      (0x07U)
#define DRV8316_REG_CTRL_6      (0x08U)
#define DRV8316_REG_CTRL_10     (0x0CU)

#define DRV8316_IC_STATUS_BK_FLT   (1U << 6)
#define DRV8316_IC_STATUS_SPI_FLT  (1U << 5)
#define DRV8316_IC_STATUS_OCP      (1U << 4)
#define DRV8316_IC_STATUS_NPOR     (1U << 3)
#define DRV8316_IC_STATUS_OVP      (1U << 2)
#define DRV8316_IC_STATUS_OT       (1U << 1)
#define DRV8316_IC_STATUS_FAULT    (1U << 0)

#define DRV8316_STATUS1_OTW        (1U << 7)
#define DRV8316_STATUS1_OTS        (1U << 6)
#define DRV8316_STATUS1_OCP_HC     (1U << 5)
#define DRV8316_STATUS1_OCP_LC     (1U << 4)
#define DRV8316_STATUS1_OCP_HB     (1U << 3)
#define DRV8316_STATUS1_OCP_LB     (1U << 2)
#define DRV8316_STATUS1_OCP_HA     (1U << 1)
#define DRV8316_STATUS1_OCP_LA     (1U << 0)

#define DRV8316_STATUS2_OTP_ERR       (1U << 6)
#define DRV8316_STATUS2_BUCK_OCP      (1U << 5)
#define DRV8316_STATUS2_BUCK_UV       (1U << 4)
#define DRV8316_STATUS2_VCP_UV        (1U << 3)
#define DRV8316_STATUS2_SPI_PARITY    (1U << 2)
#define DRV8316_STATUS2_SPI_SCLK_FLT  (1U << 1)
#define DRV8316_STATUS2_SPI_ADDR_FLT  (1U << 0)

typedef enum {
  DRV8316_PWM_MODE_6X = 0U,
  DRV8316_PWM_MODE_6X_CURRENT_LIMIT = 1U,
  DRV8316_PWM_MODE_3X = 2U,
  DRV8316_PWM_MODE_3X_CURRENT_LIMIT = 3U
} drv8316_pwm_mode_t;

typedef enum {
  DRV8316_SLEW_25V_US = 0U,
  DRV8316_SLEW_50V_US = 1U,
  DRV8316_SLEW_125V_US = 2U,
  DRV8316_SLEW_200V_US = 3U
} drv8316_slew_t;

typedef enum {
  DRV8316_CSA_GAIN_0P15_V_PER_A = 0U,
  DRV8316_CSA_GAIN_0P30_V_PER_A = 1U,
  DRV8316_CSA_GAIN_0P60_V_PER_A = 2U,
  DRV8316_CSA_GAIN_1P20_V_PER_A = 3U
} drv8316_csa_gain_t;

typedef enum {
  DRV8316_OCP_DEG_0P2_US = 0U,
  DRV8316_OCP_DEG_0P6_US = 1U,
  DRV8316_OCP_DEG_1P25_US = 2U,
  DRV8316_OCP_DEG_1P6_US = 3U
} drv8316_ocp_deglitch_t;

typedef enum {
  DRV8316_OCP_MODE_LATCHED = 0U,
  DRV8316_OCP_MODE_AUTO_RETRY = 1U,
  DRV8316_OCP_MODE_REPORT_ONLY = 2U,
  DRV8316_OCP_MODE_DISABLED = 3U
} drv8316_ocp_mode_t;

typedef struct {
  drv8316_pwm_mode_t pwm_mode;
  drv8316_slew_t slew;
  drv8316_csa_gain_t csa_gain;
  drv8316_ocp_deglitch_t ocp_deglitch;
  drv8316_ocp_mode_t ocp_mode;
  uint8_t ocp_retry_500ms;
  uint8_t ocp_level_24a;
  uint8_t enable_active_rectification;
  uint8_t enable_delay_compensation;
  uint8_t delay_target;
} drv8316_config_t;

typedef struct {
  uint8_t spi_status;
  uint8_t ic_status;
  uint8_t status1;
  uint8_t status2;
} drv8316_status_t;

void drv8316_get_default_config(drv8316_config_t *config);
HAL_StatusTypeDef drv8316_init(const drv8316_config_t *config);
HAL_StatusTypeDef drv8316_configure(const drv8316_config_t *config);
HAL_StatusTypeDef drv8316_read_register(uint8_t address, uint8_t *data, uint8_t *spi_status);
HAL_StatusTypeDef drv8316_write_register(uint8_t address, uint8_t data, uint8_t *spi_status);
HAL_StatusTypeDef drv8316_read_status(drv8316_status_t *status);
HAL_StatusTypeDef drv8316_clear_faults(void);
float drv8316_csa_gain_volts_per_amp(drv8316_csa_gain_t gain);
uint8_t drv8316_status_has_fault(const drv8316_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
