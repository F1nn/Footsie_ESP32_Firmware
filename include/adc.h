#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_adc/adc_continuous.h"

/* Initialize continuous ADC for given channels. Returns handle in out_handle. */
void adc_continuous_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);

/* Initialize ADC calibration (curve fitting). */
void adc_calibration_init(void);

/* Convert a raw ADC sample to millivolts using calibration when available. */
esp_err_t adc_raw_to_voltage(uint32_t raw, int *millivolts);

/* Map measured ADC mV to curved DAC mV using current gamma and output scaling. */
uint32_t adc_mV_to_curved_dac_mV(uint32_t adc_mV);

/* Return true if calibration handle is available (curve-fitting). */
bool adc_has_calibration(void);

#endif /* ADC_H */
