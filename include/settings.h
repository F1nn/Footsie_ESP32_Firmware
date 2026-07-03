#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include "esp_err.h"

/* Initialize settings state by loading persisted values from NVS.
 * Note: `nvs_flash_init()` must be called before this function. */
void settings_load_from_nvs(void);

/* Return gamma as stored multiplied by 100 (e.g. 220 = 2.20) */
uint16_t settings_get_gamma_x100(void);

/* Return gamma as float */
float settings_get_gamma(void);

/* Set gamma (x100) and persist to NVS. Returns ESP_OK on success. */
esp_err_t settings_set_gamma_x100(uint16_t gamma_x100);

/* ADC calibration (millivolts) */
uint16_t settings_get_adc_min_mv(void);
uint16_t settings_get_adc_max_mv(void);
esp_err_t settings_set_adc_calibrated_mv(uint16_t min_mv, uint16_t max_mv);

/* DAC output scaling (millivolts) */
uint16_t settings_get_output_min_mv(void);
uint16_t settings_get_output_max_mv(void);
esp_err_t settings_set_output_scaled_mv(uint16_t min_mv, uint16_t max_mv);

#endif /* SETTINGS_H */
