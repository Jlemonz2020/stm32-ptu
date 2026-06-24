#include "motor_app.h"

#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"

#include "as5600.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "can_bus.h"
#include "drv8316.h"
#include "foc.h"
#include "soft_i2c.h"
#include "uart_console.h"

#define MOTOR_APP_ADC_CAL_SAMPLES      (128U)
#define MOTOR_APP_STATUS_PERIOD_MS     (100U)
#define MOTOR_APP_CAN_PERIOD_MS        (20U)

static volatile uint32_t control_ticks;
static drv8316_status_t drv8316_status;

static void handle_console_line(char *line)
{
  if (strncmp(line, "en", 2U) == 0) {
    if (foc_enable() == HAL_OK) {
      (void)uart_console_printf("motor enabled\r\n");
    } else {
      (void)uart_console_printf("motor enable failed\r\n");
    }
  } else if (strncmp(line, "dis", 3U) == 0) {
    foc_disable();
    (void)uart_console_printf("motor disabled\r\n");
  } else if (strncmp(line, "iq ", 3U) == 0) {
    foc_set_iq_target((float)atof(&line[3]));
    (void)uart_console_printf("iq target set\r\n");
  } else if (strncmp(line, "ol ", 3U) == 0) {
    foc_set_open_loop((float)atof(&line[3]), 12.0f);
    (void)uart_console_printf("open loop target set\r\n");
  } else if (strncmp(line, "drv", 3U) == 0) {
    if (drv8316_read_status(&drv8316_status) == HAL_OK) {
      (void)uart_console_printf("drv stat=%02X ic=%02X s1=%02X s2=%02X\r\n",
                                drv8316_status.spi_status,
                                drv8316_status.ic_status,
                                drv8316_status.status1,
                                drv8316_status.status2);
    } else {
      (void)uart_console_printf("drv read failed\r\n");
    }
  } else if (strncmp(line, "fltclr", 6U) == 0) {
    if (drv8316_clear_faults() == HAL_OK) {
      (void)uart_console_printf("drv faults cleared\r\n");
    } else {
      (void)uart_console_printf("drv clear failed\r\n");
    }
  }
}

void motor_app_init(void)
{
  as5600_config_t as5600_config;
  drv8316_config_t drv_config;

  bsp_pwm_init();
  bsp_adc_init();
  soft_i2c_init();
  uart_console_init();
  foc_init();

  drv8316_get_default_config(&drv_config);
  drv_config.pwm_mode = DRV8316_PWM_MODE_6X;
  drv_config.csa_gain = DRV8316_CSA_GAIN_0P15_V_PER_A;
  if (drv8316_init(&drv_config) == HAL_OK) {
    bsp_adc_set_current_sense_gain(drv8316_csa_gain_volts_per_amp(drv_config.csa_gain));
    (void)uart_console_printf("drv8316 ready, csa=%.2fV/A\r\n",
                              drv8316_csa_gain_volts_per_amp(drv_config.csa_gain));
  } else {
    (void)uart_console_printf("drv8316 init failed\r\n");
  }

  as5600_config.pole_pairs = FOC_DEFAULT_POLE_PAIRS;
  as5600_config.zero_offset_rad = 0.0f;
  as5600_config.direction = 1;
  (void)as5600_init(&as5600_config);

  if (bsp_adc_calibrate(MOTOR_APP_ADC_CAL_SAMPLES) != HAL_OK) {
    (void)uart_console_printf("adc calibration failed\r\n");
  }

  if (can_bus_init() != HAL_OK) {
    (void)uart_console_printf("can init failed\r\n");
  }

  (void)uart_console_printf("FOC framework ready, pwm=%luHz can=1Mbps\r\n",
                            (unsigned long)BSP_PWM_FREQUENCY_HZ);
}

void motor_app_motor_task(void *argument)
{
  (void)argument;

  for (;;) {
    float mech_angle;
    if (as5600_read_mech_angle(&mech_angle) == HAL_OK) {
      foc_update_sensor_angle(mech_angle);
    }

    osDelay(1U);
  }
}

void motor_app_comm_task(void *argument)
{
  uint32_t last_status_tick = 0U;
  uint32_t last_can_tick = 0U;
  char line[96];

  (void)argument;

  for (;;) {
    uint32_t now = HAL_GetTick();
    foc_status_t status;

    uart_console_poll_rx();
    can_bus_process_rx();

    if (uart_console_get_line(line, sizeof(line)) != 0U) {
      handle_console_line(line);
    }

    if ((now - last_can_tick) >= MOTOR_APP_CAN_PERIOD_MS) {
      last_can_tick = now;
      (void)can_bus_send_status();
    }

    if ((now - last_status_tick) >= MOTOR_APP_STATUS_PERIOD_MS) {
      last_status_tick = now;
      foc_get_status(&status);
      (void)drv8316_read_status(&drv8316_status);
      (void)uart_console_printf("en=%u mode=%u iq=%.2f/%.2f vbus=%.1f temp=%.1f drv=%02X/%02X/%02X duty=%.2f %.2f %.2f\r\n",
                                status.enabled,
                                (unsigned int)status.mode,
                                status.iq,
                                status.iq_target,
                                status.vbus,
                                status.temperature,
                                drv8316_status.ic_status,
                                drv8316_status.status1,
                                drv8316_status.status2,
                                status.duty.a,
                                status.duty.b,
                                status.duty.c);
    }

    osDelay(5U);
  }
}

void motor_app_on_control_tick(void)
{
  control_ticks++;
  foc_fast_loop();
}
