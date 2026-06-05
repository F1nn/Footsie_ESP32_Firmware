// System includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "system.h"
#include "esp_ws28xx.h"
#include "dac80501.h"

#define EXAMPLE_READ_LEN                    64 // 16 samples, each sample has 4 bytes (see SOC_ADC_DIGI_RESULT_BYTES)
#define MAX_PARSED_SAMPLES                  (EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES)
/* ADC reference (in mV) for converting ADC code to millivolts */
#define ADC_VREF_MV                         3300
/* DAC full-scale output target in mV */
#define DAC_FULL_SCALE_MV                   5000
/* Maximum DAC update rate */
#define DAC_UPDATE_PERIOD_MS                25
/* ADC calibrated range (mV) - adjust these to match your potentiometer's actual measurement range */
/* Set to the calibrated ADC reading at the pot's minimum position */
#define ADC_MIN_CALIBRATED_MV               139
/* Set to the calibrated ADC reading at the pot's maximum position */
#define ADC_MAX_CALIBRATED_MV               3181

static adc_channel_t channel[1] = {ADC_CHANNEL_2}; // Channel 2 maps to GPIO3

static const char *TAG = "Footsie";

CRGB *ws2812_buffer;
dac80501_device_t dac80501_dev;
i2c_master_bus_handle_t i2c_bus_handle = NULL;
static dac80501_status_t dac_status = DAC80501_ERR_NOT_INITIALIZED;
static adc_cali_handle_t adc_cali_handle = NULL;

static adc_continuous_data_t s_parsed_data[MAX_PARSED_SAMPLES];

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000, // 20kHz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static void adc_calibration_init(void)
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

static void sys_init(void) {
    // Delay to wait for uart connection so nothing is missed.
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    char *task_name = pcTaskGetName(NULL);
    ESP_LOGI(task_name, "Initialisation Started.");

    esp_log_level_set("I2C", ESP_LOG_DEBUG);

    // I2C bus configuration
    i2c_master_bus_config_t i2c_master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    // Initialize DAC80501 at default address 0x48
    dac_status = dac80501_init(&dac80501_dev, i2c_bus_handle, 0x48);
    if (dac_status != DAC80501_OK) {
        ESP_LOGE(TAG, "Failed to initialize DAC80501: %d", dac_status);
    } else {
        ESP_LOGI(TAG, "DAC80501 initialized successfully");
        // Set DAC to 0V (zero) initially
          /* Defaults per datasheet: BUF-GAIN=1 (2x internal 2.5V ref) and REF-DIV=0 (no divider)
              mean the hardware will provide 0-5V full-scale when powered from 5V.
              The driver sets these defaults in init; just write 0 to the DAC output. */
          dac80501_write_dac(&dac80501_dev, 0x0000);
    }

    gpio_reset_pin(PIN_ADC_IN);
    gpio_set_direction(PIN_ADC_IN, GPIO_MODE_INPUT);

    // WS2812B RGB LED driver initialisation.
    if (ws28xx_init(PIN_RGB, WS2812B, 1, &ws2812_buffer) == ESP_OK) {
        // Set LED to green as init complete confirmation
        ws2812_buffer[0] = (CRGB){.r=0, .g=1, .b=0};
        ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
    } else {
        ESP_LOGW(TAG, "WS2812 init failed, skipping LED confirmation");
    }

    ESP_LOGI(task_name, "Initialisation Complete.");
}

void app_main(void) {

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    uint64_t window_sample_sum_mV = 0;
    uint32_t window_sample_count = 0;
    uint32_t last_avg_value_mV = ADC_MIN_CALIBRATED_MV;
    uint32_t last_clamped_value_mV = ADC_MIN_CALIBRATED_MV;
    int32_t last_unclamped_target_mV = 0;
    uint32_t last_target_mV = 0;
    TickType_t last_dac_update_tick = 0;
    const TickType_t dac_update_period_ticks = pdMS_TO_TICKS(DAC_UPDATE_PERIOD_MS);

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    adc_calibration_init();

    ESP_LOGI(TAG, "Scale config: adc_min=%u mV, adc_max=%u mV, adc_span=%u mV, dac_full_scale=%u mV",
             ADC_MIN_CALIBRATED_MV,
             ADC_MAX_CALIBRATED_MV,
             (unsigned)(ADC_MAX_CALIBRATED_MV - ADC_MIN_CALIBRATED_MV),
             DAC_FULL_SCALE_MV);
    ESP_LOGI(TAG, "ADC config: vref=%u mV, attenuation=%d, calibration=%s",
             ADC_VREF_MV,
             ADC_ATTEN_DB_12,
             (adc_cali_handle != NULL) ? "curve-fitting" : "fallback-linear");
    ESP_LOGI(TAG, "Update rate: period=%u ms", DAC_UPDATE_PERIOD_MS);

    // Start ADC reading continuously
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    sys_init();

    // Output to DAC80501 only after a successful init.
    if (dac_status == DAC80501_OK) {
        /* write 0mV using device default vref (pass 0 to use device vref) */
        if (dac80501_write_voltage(&dac80501_dev, 0, 0) != DAC80501_OK) {
            ESP_LOGW(TAG, "Failed to write 0mV to DAC");
        }
    }

    while (1) {
        uint32_t num_parsed_samples = 0;

        ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) {
            esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, s_parsed_data, &num_parsed_samples);
            if (parse_ret == ESP_OK) {
                for (int i = 0; i < num_parsed_samples; i++) {
                    if (s_parsed_data[i].valid) {
                        int calibrated_mv = 0;
                        if (adc_cali_handle != NULL) {
                            if (adc_cali_raw_to_voltage(adc_cali_handle, s_parsed_data[i].raw_data, &calibrated_mv) != ESP_OK) {
                                calibrated_mv = (int)(((uint64_t)s_parsed_data[i].raw_data * ADC_VREF_MV + 2047) / 4095);
                            }
                        } else {
                            calibrated_mv = (int)(((uint64_t)s_parsed_data[i].raw_data * ADC_VREF_MV + 2047) / 4095);
                        }

                        // Accumulate all samples within the current update window.
                        window_sample_sum_mV += (uint32_t)calibrated_mv;
                        window_sample_count++;
                    } else {
                        ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
                                 s_parsed_data[i].unit + 1,
                                 s_parsed_data[i].channel,
                                 s_parsed_data[i].raw_data);
                    }
                }
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (last_dac_update_tick == 0 || (now - last_dac_update_tick) >= dac_update_period_ticks) {
            uint32_t samples_used = window_sample_count;

            if (window_sample_count > 0) {
                uint32_t avg_value = (uint32_t)(window_sample_sum_mV / window_sample_count);
                last_avg_value_mV = avg_value;

                // Clamp averaged reading to the calibrated measurement range.
                uint32_t clamped_value = avg_value;
                if (clamped_value < ADC_MIN_CALIBRATED_MV) {
                    clamped_value = ADC_MIN_CALIBRATED_MV;
                }
                if (clamped_value > ADC_MAX_CALIBRATED_MV) {
                    clamped_value = ADC_MAX_CALIBRATED_MV;
                }

                last_clamped_value_mV = clamped_value;
                last_unclamped_target_mV = (int32_t)(((int64_t)((int32_t)avg_value - (int32_t)ADC_MIN_CALIBRATED_MV) * DAC_FULL_SCALE_MV) /
                                                     ((int32_t)ADC_MAX_CALIBRATED_MV - (int32_t)ADC_MIN_CALIBRATED_MV));
                last_target_mV = (uint32_t)(((uint64_t)(clamped_value - ADC_MIN_CALIBRATED_MV) * DAC_FULL_SCALE_MV) /
                                            (ADC_MAX_CALIBRATED_MV - ADC_MIN_CALIBRATED_MV));
            }

            ws2812_buffer[0] = (CRGB){.r=0, .g=last_clamped_value_mV / 16, .b=0};
            ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());

            if (dac80501_write_voltage(&dac80501_dev, last_target_mV, 0) != DAC80501_OK) {
                ESP_LOGW(TAG, "Failed to write %u mV to DAC", last_target_mV);
            }

            ESP_LOGI(TAG, "Update: samples=%5u, avg_u=%5u mV, avg_c=%5u mV, target_u=%6"PRId32" mV, target_c=%5u mV",
                     samples_used,
                     last_avg_value_mV,
                     last_clamped_value_mV,
                     last_unclamped_target_mV,
                     last_target_mV);

            window_sample_sum_mV = 0;
            window_sample_count = 0;
            last_dac_update_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));

    if (adc_cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
    }

    // vTaskDelay(100 / portTICK_PERIOD_MS);
}