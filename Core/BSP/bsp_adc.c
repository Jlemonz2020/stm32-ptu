#include "bsp_adc.h"

#include <math.h>

#include "adc.h"

typedef struct {
  float phase_a_offset;
  float phase_b_offset;
  float phase_c_offset;
  float current_v_per_a;
  uint8_t calibrated;
} bsp_adc_state_t;

static bsp_adc_state_t adc_state = {
  2048.0f,
  2048.0f,
  2048.0f,
  BSP_ADC_CURRENT_V_PER_A,
  0U
};

static HAL_StatusTypeDef read_adc_channel(ADC_HandleTypeDef *hadc,
                                          uint32_t channel,
                                          uint16_t *value)
{
  ADC_ChannelConfTypeDef config;
  HAL_StatusTypeDef status;

  if (value == NULL) {
    return HAL_ERROR;
  }

  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;

  status = HAL_ADC_ConfigChannel(hadc, &config);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_ADC_Start(hadc);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_ADC_PollForConversion(hadc, 2U);
  if (status == HAL_OK) {
    *value = (uint16_t)HAL_ADC_GetValue(hadc);
  }

  (void)HAL_ADC_Stop(hadc);
  return status;
}

static float raw_to_voltage(uint16_t raw)
{
  return ((float)raw * BSP_ADC_VREF) / BSP_ADC_FULL_SCALE;
}

static float raw_to_current(uint16_t raw, float offset)
{
  float voltage_delta = (((float)raw - offset) * BSP_ADC_VREF) / BSP_ADC_FULL_SCALE;
  return voltage_delta / adc_state.current_v_per_a;
}

static float ntc_temperature_c_from_voltage(float voltage)
{
  float resistance;
  float inv_temp_k;

  if ((voltage <= 0.0f) || (voltage >= BSP_ADC_VREF)) {
    return 0.0f;
  }

  resistance = (BSP_ADC_NTC_PULLUP_OHM * voltage) / (BSP_ADC_VREF - voltage);
  inv_temp_k = (1.0f / BSP_ADC_NTC_NOMINAL_TEMP_K) +
               (logf(resistance / BSP_ADC_NTC_NOMINAL_OHM) / BSP_ADC_NTC_BETA);
  return (1.0f / inv_temp_k) - 273.15f;
}

void bsp_adc_init(void)
{
  (void)HAL_ADC_Stop(&hadc1);
  (void)HAL_ADC_Stop(&hadc2);
  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  (void)HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  adc_state.current_v_per_a = BSP_ADC_CURRENT_V_PER_A;
}

void bsp_adc_set_current_sense_gain(float volts_per_amp)
{
  if (volts_per_amp <= 0.0f) {
    volts_per_amp = BSP_ADC_CURRENT_V_PER_A;
  }
  adc_state.current_v_per_a = volts_per_amp;
}

HAL_StatusTypeDef bsp_adc_calibrate(uint16_t samples)
{
  uint32_t sum_a = 0U;
  uint32_t sum_b = 0U;
  uint32_t sum_c = 0U;
  uint16_t raw = 0U;
  uint16_t i;
  HAL_StatusTypeDef status;

  if (samples == 0U) {
    samples = 1U;
  }

  for (i = 0U; i < samples; i++) {
    status = read_adc_channel(&hadc1, ADC_CHANNEL_1, &raw);
    if (status != HAL_OK) {
      return status;
    }
    sum_a += raw;

    status = read_adc_channel(&hadc1, ADC_CHANNEL_2, &raw);
    if (status != HAL_OK) {
      return status;
    }
    sum_b += raw;

    status = read_adc_channel(&hadc1, ADC_CHANNEL_3, &raw);
    if (status != HAL_OK) {
      return status;
    }
    sum_c += raw;
  }

  adc_state.phase_a_offset = (float)sum_a / (float)samples;
  adc_state.phase_b_offset = (float)sum_b / (float)samples;
  adc_state.phase_c_offset = (float)sum_c / (float)samples;
  adc_state.calibrated = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef bsp_adc_get_raw(bsp_adc_raw_t *raw)
{
  HAL_StatusTypeDef status;

  if (raw == NULL) {
    return HAL_ERROR;
  }

  status = read_adc_channel(&hadc1, ADC_CHANNEL_1, &raw->phase_a);
  if (status != HAL_OK) {
    return status;
  }
  status = read_adc_channel(&hadc1, ADC_CHANNEL_2, &raw->phase_b);
  if (status != HAL_OK) {
    return status;
  }
  status = read_adc_channel(&hadc1, ADC_CHANNEL_3, &raw->phase_c);
  if (status != HAL_OK) {
    return status;
  }
  status = read_adc_channel(&hadc1, ADC_CHANNEL_4, &raw->vbus);
  if (status != HAL_OK) {
    return status;
  }
  status = read_adc_channel(&hadc2, ADC_CHANNEL_17, &raw->temp);
  return status;
}

HAL_StatusTypeDef bsp_adc_get_phase_currents(bsp_phase_currents_t *currents)
{
  uint16_t raw_a;
  uint16_t raw_b;
  uint16_t raw_c;
  HAL_StatusTypeDef status;

  if (currents == NULL) {
    return HAL_ERROR;
  }

  status = read_adc_channel(&hadc1, ADC_CHANNEL_1, &raw_a);
  if (status != HAL_OK) {
    return status;
  }
  status = read_adc_channel(&hadc1, ADC_CHANNEL_2, &raw_b);
  if (status != HAL_OK) {
    return status;
  }
  status = read_adc_channel(&hadc1, ADC_CHANNEL_3, &raw_c);
  if (status != HAL_OK) {
    return status;
  }

  currents->ia = raw_to_current(raw_a, adc_state.phase_a_offset);
  currents->ib = raw_to_current(raw_b, adc_state.phase_b_offset);
  currents->ic = raw_to_current(raw_c, adc_state.phase_c_offset);
  return HAL_OK;
}

float bsp_adc_get_vbus(void)
{
  uint16_t raw = 0U;

  if (read_adc_channel(&hadc1, ADC_CHANNEL_4, &raw) != HAL_OK) {
    return 0.0f;
  }

  return raw_to_voltage(raw) * BSP_ADC_VBUS_DIVIDER_GAIN;
}

float bsp_adc_get_temp(void)
{
  uint16_t raw = 0U;
  float voltage;

  if (read_adc_channel(&hadc2, ADC_CHANNEL_17, &raw) != HAL_OK) {
    return 0.0f;
  }

  voltage = raw_to_voltage(raw);
  return ntc_temperature_c_from_voltage(voltage);
}
