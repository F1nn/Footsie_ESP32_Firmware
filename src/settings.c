#include "settings.h"
#include "nvs.h"
#include "esp_log.h"
#include "system.h"

static const char *TAG = "settings";

static volatile float s_output_curve_gamma = OUTPUT_CURVE_GAMMA;
static volatile uint16_t s_adc_min_calibrated_mv = ADC_MIN_CALIBRATED_MV;
static volatile uint16_t s_adc_max_calibrated_mv = ADC_MAX_CALIBRATED_MV;

void settings_load_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace (%s), using default gamma", esp_err_to_name(err));
        s_output_curve_gamma = OUTPUT_CURVE_GAMMA;
        return;
    }

    uint16_t gamma_x100 = 0;
    err = nvs_get_u16(nvs_handle, NVS_KEY_GAMMA, &gamma_x100);

    /* Read ADC calibration values (millivolts) */
    uint16_t adc_min = ADC_MIN_CALIBRATED_MV;
    uint16_t adc_max = ADC_MAX_CALIBRATED_MV;
    esp_err_t err_min = nvs_get_u16(nvs_handle, NVS_KEY_ADC_MIN_MV, &adc_min);
    esp_err_t err_max = nvs_get_u16(nvs_handle, NVS_KEY_ADC_MAX_MV, &adc_max);

    nvs_close(nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "Gamma not found in NVS, using default");
        s_output_curve_gamma = OUTPUT_CURVE_GAMMA;
        return;
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read gamma from NVS (%s), using default", esp_err_to_name(err));
        s_output_curve_gamma = OUTPUT_CURVE_GAMMA;
        return;
    }

    if (gamma_x100 < OUTPUT_CURVE_GAMMA_MIN_X100 || gamma_x100 > OUTPUT_CURVE_GAMMA_MAX_X100) {
        ESP_LOGW(TAG, "Stored gamma %u out of range, using default", gamma_x100);
        s_output_curve_gamma = OUTPUT_CURVE_GAMMA;
        return;
    }

    s_output_curve_gamma = (float)gamma_x100 / 100.0f;
    ESP_LOGI(TAG, "Gamma loaded from NVS: %u.%02u", gamma_x100 / 100u, gamma_x100 % 100u);

    /* Validate and apply ADC calibration values read from NVS */
    if (err_min == ESP_OK && err_max == ESP_OK) {
        if (adc_min >= adc_max) {
            ESP_LOGW(TAG, "Stored ADC calibration invalid (min >= max), using defaults");
            s_adc_min_calibrated_mv = ADC_MIN_CALIBRATED_MV;
            s_adc_max_calibrated_mv = ADC_MAX_CALIBRATED_MV;
        } else {
            s_adc_min_calibrated_mv = adc_min;
            s_adc_max_calibrated_mv = adc_max;
            ESP_LOGI(TAG, "ADC calibration loaded from NVS: min=%u mV max=%u mV", adc_min, adc_max);
        }
    } else {
        ESP_LOGD(TAG, "ADC calibration not found in NVS, using defaults");
        s_adc_min_calibrated_mv = ADC_MIN_CALIBRATED_MV;
        s_adc_max_calibrated_mv = ADC_MAX_CALIBRATED_MV;
    }
}

uint16_t settings_get_gamma_x100(void)
{
    return (uint16_t)((s_output_curve_gamma * 100.0f) + 0.5f);
}

float settings_get_gamma(void)
{
    return s_output_curve_gamma;
}

esp_err_t settings_set_gamma_x100(uint16_t gamma_x100)
{
    if (gamma_x100 < OUTPUT_CURVE_GAMMA_MIN_X100 || gamma_x100 > OUTPUT_CURVE_GAMMA_MAX_X100) {
        ESP_LOGE(TAG, "Cannot save gamma %u: out of range [%u, %u]", gamma_x100, OUTPUT_CURVE_GAMMA_MIN_X100, OUTPUT_CURVE_GAMMA_MAX_X100);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing (%s)", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u16(nvs_handle, NVS_KEY_GAMMA, gamma_x100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write gamma to NVS (%s)", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit gamma to NVS (%s)", esp_err_to_name(err));
        return err;
    }

    s_output_curve_gamma = (float)gamma_x100 / 100.0f;
    ESP_LOGD(TAG, "Gamma saved to NVS: %u.%02u", gamma_x100 / 100u, gamma_x100 % 100u);
    return ESP_OK;
}

uint16_t settings_get_adc_min_mv(void)
{
    return s_adc_min_calibrated_mv;
}

uint16_t settings_get_adc_max_mv(void)
{
    return s_adc_max_calibrated_mv;
}

esp_err_t settings_set_adc_calibrated_mv(uint16_t min_mv, uint16_t max_mv)
{
    if (min_mv >= max_mv) {
        ESP_LOGE(TAG, "Invalid ADC calibration: min (%u) >= max (%u)", min_mv, max_mv);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing (%s)", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u16(nvs_handle, NVS_KEY_ADC_MIN_MV, min_mv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ADC min to NVS (%s)", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_set_u16(nvs_handle, NVS_KEY_ADC_MAX_MV, max_mv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ADC max to NVS (%s)", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit ADC calibration to NVS (%s)", esp_err_to_name(err));
        return err;
    }

    s_adc_min_calibrated_mv = min_mv;
    s_adc_max_calibrated_mv = max_mv;
    ESP_LOGI(TAG, "ADC calibration saved to NVS: min=%u mV max=%u mV", min_mv, max_mv);
    return ESP_OK;
}
