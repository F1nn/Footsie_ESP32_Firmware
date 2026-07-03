#include "adc.h"

#include <math.h>

#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"
#include "settings.h"
#include "system.h"

static const char *TAG = "adc";
static adc_cali_handle_t adc_cali_handle = NULL;

bool adc_has_calibration(void)
{
    return adc_cali_handle != NULL;
}

void adc_continuous_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void adc_calibration_init(void)
{
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_2,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration initialized");
    } else {
        ESP_LOGW(TAG, "ADC calibration init failed: %s", esp_err_to_name(ret));
        adc_cali_handle = NULL;
    }
}

esp_err_t adc_raw_to_voltage(uint32_t raw, int *millivolts)
{
    if (millivolts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (adc_cali_handle != NULL) {
        esp_err_t ret = adc_cali_raw_to_voltage(adc_cali_handle, raw, millivolts);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
    }

    *millivolts = (int)(((uint64_t)raw * ADC_VREF_MV + 2047) / 4095);
    return ESP_OK;
}

uint32_t adc_mV_to_curved_dac_mV(uint32_t adc_mV)
{
    if (ADC_SPAN_MV <= 0) {
        return 0;
    }

    if (adc_mV <= ADC_MIN_CALIBRATED_MV) {
        return 0;
    }

    if (adc_mV >= ADC_MAX_CALIBRATED_MV) {
        return DAC_FULL_SCALE_MV;
    }

    float normalized = (float)(adc_mV - ADC_MIN_CALIBRATED_MV) / (float)ADC_SPAN_MV;
    if (normalized < 0.0f) {
        normalized = 0.0f;
    } else if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    float gamma = settings_get_gamma();
    float curved = powf(normalized, gamma);
    uint32_t output_mV = (uint32_t)((curved * (float)DAC_FULL_SCALE_MV) + 0.5f);
    if (output_mV > DAC_FULL_SCALE_MV) {
        output_mV = DAC_FULL_SCALE_MV;
    }

    return output_mV;
}
