// System includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "system.h"
#include "esp_ws28xx.h"
#include "dac80501.h"
#include "adc.h"
#include "ble.h"
#include "settings.h"
#include "esp_task_wdt.h"

static adc_channel_t channel[1] = {ADC_CHANNEL_2}; // Channel 2 maps to GPIO3

static const char *TAG = "Footsie";

CRGB *ws2812_buffer;
static bool ws2812_ready = false;
dac80501_device_t dac80501_dev;
i2c_master_bus_handle_t i2c_bus_handle = NULL;
static dac80501_status_t dac_status = DAC80501_ERR_NOT_INITIALIZED;

static adc_continuous_data_t s_parsed_data[MAX_PARSED_SAMPLES];
static volatile uint16_t s_latest_adc_raw = 0;

/* BLE and ADC helpers moved into their own modules: see /src/ble.c and /src/adc.c */

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
        dac80501_write_dac(&dac80501_dev, 0x0000);
    }

    gpio_reset_pin(PIN_ADC_IN);
    gpio_set_direction(PIN_ADC_IN, GPIO_MODE_INPUT);

    // WS2812B RGB LED driver initialisation.
    if (ws28xx_init(PIN_RGB, WS2812B, 1, &ws2812_buffer) == ESP_OK) {
        // Set LED to green as init complete confirmation
        ws2812_ready = true;
        ws2812_buffer[0] = (CRGB){.r=0, .g=1, .b=0};
        ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
    } else {
        ESP_LOGW(TAG, "WS2812 init failed, skipping LED confirmation");
    }

    ble_init();

    settings_load_from_nvs();
    ESP_LOGI(TAG, "Output curve gamma: %0.3f", (double)settings_get_gamma());

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
    TickType_t last_debug_log_tick = 0;
    const TickType_t dac_update_period_ticks = pdMS_TO_TICKS(DAC_UPDATE_PERIOD_MS);
    const TickType_t debug_log_period_ticks = pdMS_TO_TICKS(DEBUG_LOG_PERIOD_MS);

    adc_continuous_handle_t handle = NULL;
    adc_continuous_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    adc_calibration_init();

    ESP_LOGI(TAG, "Scale config: adc_min=%u mV, adc_max=%u mV, adc_span=%u mV, dac_full_scale=%u mV",
             ADC_MIN_CALIBRATED_MV,
             ADC_MAX_CALIBRATED_MV,
             (unsigned)(ADC_MAX_CALIBRATED_MV - ADC_MIN_CALIBRATED_MV),
             DAC_FULL_SCALE_MV);
    ESP_LOGI(TAG, "ADC config: vref=%u mV, attenuation=%d, calibration=%s",
             ADC_VREF_MV,
             ADC_ATTEN_DB_12,
             adc_has_calibration() ? "curve-fitting" : "fallback-linear");
    ESP_LOGI(TAG, "Update rate: period=%u ms", DAC_UPDATE_PERIOD_MS);

    // Start ADC reading continuously
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    sys_init();

    /* Subscribe this task to the already-configured task watchdog.
     * The app startup code initializes TWDT for the idle tasks, so we only
     * need to add the main task and keep resetting it while busy. */
    esp_task_wdt_add(NULL);

    // Output to DAC80501 only after a successful init.
    if (dac_status == DAC80501_OK) {
        /* write 0mV using device default vref (pass 0 to use device vref) */
        if (dac80501_write_voltage(&dac80501_dev, 0, 0) != DAC80501_OK) {
            ESP_LOGW(TAG, "Failed to write 0mV to DAC");
        }
    }

    while (1) {
        /* Keep task watchdog happy while doing work in this loop. */
        esp_task_wdt_reset();
        uint32_t num_parsed_samples = 0;
        bool adc_read_valid = false;
        bool adc_read_timed_out = false;

        ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, pdMS_TO_TICKS(5));
        if (ret == ESP_OK) {
            adc_read_valid = true;
            esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, s_parsed_data, &num_parsed_samples);
            if (parse_ret == ESP_OK) {
                        if (num_parsed_samples > MAX_PARSED_SAMPLES) {
                            ESP_LOGW(TAG, "Parsed samples (%u) exceed buffer (%u), clipping", num_parsed_samples, MAX_PARSED_SAMPLES);
                            num_parsed_samples = MAX_PARSED_SAMPLES;
                        }
                for (uint32_t i = 0; i < num_parsed_samples; i++) {
                    if (s_parsed_data[i].valid) {
                        int calibrated_mv = 0;
                        s_latest_adc_raw = (uint16_t)s_parsed_data[i].raw_data;

                        if (adc_raw_to_voltage(s_parsed_data[i].raw_data, &calibrated_mv) != ESP_OK) {
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
        } else if (ret == ESP_ERR_TIMEOUT) {
            adc_read_timed_out = true;
        }

        ble_poll();

        if (adc_read_timed_out && window_sample_count == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if (last_dac_update_tick == 0 || (now - last_dac_update_tick) >= dac_update_period_ticks) {
            uint32_t samples_used = window_sample_count;
            bool update_valid = false;
            float last_normalized_value = 0.0f;
            float last_curved_value = 0.0f;

                if (window_sample_count > 0) {
                    uint32_t avg_value = (uint32_t)(window_sample_sum_mV / window_sample_count);
                    last_avg_value_mV = avg_value;

                    // Send the filtered averaged value over BLE
                    ble_set_latest_adc_mV((uint16_t)avg_value);

                    /* Clamp averaged reading to the calibrated measurement range. */
                    uint32_t clamped_value = avg_value;
                    if (clamped_value < ADC_MIN_CALIBRATED_MV) {
                        clamped_value = ADC_MIN_CALIBRATED_MV;
                    }
                    if (clamped_value > ADC_MAX_CALIBRATED_MV) {
                        clamped_value = ADC_MAX_CALIBRATED_MV;
                    }

                    last_clamped_value_mV = clamped_value;
                    if (ADC_SPAN_MV <= 0) {
                        ESP_LOGW(TAG, "Invalid ADC calibration span (denom=0), skipping mapping to DAC");
                        last_unclamped_target_mV = 0;
                        last_target_mV = 0;
                    } else {
                        last_normalized_value = (float)(clamped_value - ADC_MIN_CALIBRATED_MV) / (float)ADC_SPAN_MV;
                        if (last_normalized_value < 0.0f) {
                            last_normalized_value = 0.0f;
                        } else if (last_normalized_value > 1.0f) {
                            last_normalized_value = 1.0f;
                        }
                        last_curved_value = powf(last_normalized_value, settings_get_gamma());
                        last_unclamped_target_mV = (int32_t)(((int64_t)((int32_t)avg_value - (int32_t)ADC_MIN_CALIBRATED_MV) * DAC_FULL_SCALE_MV) / (int32_t)ADC_SPAN_MV);
                        last_target_mV = adc_mV_to_curved_dac_mV(clamped_value);
                        update_valid = true;
                    }
                }

            if (ws2812_ready && ws2812_buffer != NULL) {
                uint8_t led_green = 0;
                if (update_valid) {
                    led_green = (uint8_t)(((uint64_t)last_target_mV * 255u + (DAC_FULL_SCALE_MV / 2u)) / DAC_FULL_SCALE_MV);
                }

                uint8_t led_blue = ble_is_connected() ? 8 : 0;
                ws2812_buffer[0] = (CRGB){.r=0, .g=led_green, .b=led_blue};
                ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
            }

            if (update_valid && dac_status == DAC80501_OK) {
                if (dac80501_write_voltage(&dac80501_dev, last_target_mV, 0) != DAC80501_OK) {
                    ESP_LOGW(TAG, "Failed to write %u mV to DAC", last_target_mV);
                }
            } else {
                if (dac_status == DAC80501_OK && dac80501_write_voltage(&dac80501_dev, 0, 0) != DAC80501_OK) {
                    ESP_LOGW(TAG, "Failed to write 0mV to DAC during fail-safe handling");
                }
                if (!adc_read_valid) {
                    ESP_LOGW(TAG, "ADC read failed, driving DAC to zero");
                } else {
                    ESP_LOGW(TAG, "No valid ADC update, driving DAC to zero");
                }
            }

            uint32_t applied_target_mV = update_valid ? last_target_mV : 0;
            int32_t curve_delta_mV = (int32_t)applied_target_mV - last_unclamped_target_mV;

            if (last_debug_log_tick == 0 || (now - last_debug_log_tick) >= debug_log_period_ticks) {
                ESP_LOGI(TAG, "Map: s=%u avg=%u in=%u lin=%" PRId32 " out=%u d=%+" PRId32 " n=%0.3f g=%0.3f",
                         samples_used,
                         last_avg_value_mV,
                         last_clamped_value_mV,
                         last_unclamped_target_mV,
                         applied_target_mV,
                         curve_delta_mV,
                         (double)last_normalized_value,
                         (double)last_curved_value);
                last_debug_log_tick = now;
            }

            ble_notify_latest_adc_mV();

            window_sample_sum_mV = 0;
            window_sample_count = 0;
            last_dac_update_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

}