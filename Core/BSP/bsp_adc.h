#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_ADC_VREF                 (3.3f)
#define BSP_ADC_FULL_SCALE           (4095.0f)
#define BSP_ADC_CURRENT_V_PER_A      (0.150f)
#define BSP_ADC_VBUS_DIVIDER_GAIN    (6.0f)
#define BSP_ADC_NTC_PULLUP_OHM        (10000.0f)
#define BSP_ADC_NTC_NOMINAL_OHM       (10000.0f)
#define BSP_ADC_NTC_BETA              (3950.0f)
#define BSP_ADC_NTC_NOMINAL_TEMP_K    (298.15f)

typedef struct {
  float ia;
  float ib;
  float ic;
} bsp_phase_currents_t;

typedef struct {
  uint16_t phase_a;
  uint16_t phase_b;
  uint16_t phase_c;
  uint16_t vbus;
  uint16_t temp;
} bsp_adc_raw_t;

void bsp_adc_init(void);
void bsp_adc_set_current_sense_gain(float volts_per_amp);
HAL_StatusTypeDef bsp_adc_calibrate(uint16_t samples);
HAL_StatusTypeDef bsp_adc_get_phase_currents(bsp_phase_currents_t *currents);
HAL_StatusTypeDef bsp_adc_get_raw(bsp_adc_raw_t *raw);
float bsp_adc_get_vbus(void);
float bsp_adc_get_temp(void);

#ifdef __cplusplus
}
#endif

#endif
