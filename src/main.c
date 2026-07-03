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
#include "esp_timer.h"
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
static adc_continuous_handle_t adc_handle = NULL;

/* Control update structure passed from ADC task to control task */
typedef struct {
    uint32_t avg_value_mV;
    uint32_t samples_used;
    bool adc_read_valid;
} control_update_t;

static QueueHandle_t control_queue = NULL;

static uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static int32_t i32_abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

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

/* Control task: handles DAC writes, LED updates, and logging (runs on core 1) */
static void control_task(void *pvParameters)
{
    (void)pvParameters;
    const char *task_name = pcTaskGetName(NULL);
    ESP_LOGI(task_name, "Control task started on core %d", xPortGetCoreID());
    
    esp_task_wdt_add(NULL);
    
    TickType_t last_map_log_tick = 0;
    TickType_t map_log_min_period_ticks = pdMS_TO_TICKS(MAP_LOG_MIN_PERIOD_MS);
    if (map_log_min_period_ticks == 0) {
        map_log_min_period_ticks = 1;
    }
    TickType_t last_ble_poll_tick = 0;
    const TickType_t ble_poll_period_ticks = pdMS_TO_TICKS(BLE_POLL_PERIOD_MS);
    uint32_t last_written_target_mV = UINT32_MAX;
    uint8_t last_led_green = UINT8_MAX;
    uint8_t last_led_blue = UINT8_MAX;
#if ENABLE_PERF_LOGS
    TickType_t last_perf_log_tick = 0;
    TickType_t perf_log_period_ticks = pdMS_TO_TICKS(DEBUG_LOG_PERIOD_MS);
    if (perf_log_period_ticks == 0) {
        perf_log_period_ticks = 1;
    }
    int64_t loop_us_max = 0;
    int64_t work_us_max = 0;
#endif
    uint32_t last_logged_avg_value_mV = UINT32_MAX;
    uint32_t last_logged_clamped_value_mV = UINT32_MAX;
    int32_t last_logged_unclamped_target_mV = INT32_MIN;
    uint32_t last_logged_applied_target_mV = UINT32_MAX;
    int32_t last_logged_curve_delta_mV = INT32_MIN;
    control_update_t update;
    
    while (1) {
    #if ENABLE_PERF_LOGS
        int64_t loop_start_us = esp_timer_get_time();
    #endif
        esp_task_wdt_reset();

        TickType_t now = xTaskGetTickCount();
        if (last_ble_poll_tick == 0 || (now - last_ble_poll_tick) >= ble_poll_period_ticks) {
            ble_poll();
            last_ble_poll_tick = now;
        }
        
        /* Wait for an update from the ADC task, with timeout to keep watchdog happy */
        if (xQueueReceive(control_queue, &update, pdMS_TO_TICKS(100)) != pdTRUE) {
            /* No update received, continue */
            vTaskDelay(pdMS_TO_TICKS(5));
#if ENABLE_PERF_LOGS
            int64_t loop_us = esp_timer_get_time() - loop_start_us;
            if (loop_us > loop_us_max) {
                loop_us_max = loop_us;
            }
#endif
            continue;
        }

#if ENABLE_PERF_LOGS
        int64_t work_start_us = esp_timer_get_time();
#endif

        uint32_t clamped_value_mV = update.avg_value_mV;
        if (clamped_value_mV < ADC_MIN_CALIBRATED_MV) {
            clamped_value_mV = ADC_MIN_CALIBRATED_MV;
        }
        if (clamped_value_mV > ADC_MAX_CALIBRATED_MV) {
            clamped_value_mV = ADC_MAX_CALIBRATED_MV;
        }

        bool update_valid = (update.samples_used > 0 && ADC_SPAN_MV > 0);
        uint32_t target_mV = 0;
        int32_t unclamped_target_mV = 0;
        if (update_valid) {
            uint16_t output_min_mv = settings_get_output_min_mv();
            uint16_t output_max_mv = settings_get_output_max_mv();
            uint32_t output_span_mv = (output_max_mv > output_min_mv)
                                          ? ((uint32_t)output_max_mv - (uint32_t)output_min_mv)
                                          : 0;
            unclamped_target_mV = (int32_t)output_min_mv +
                                  (int32_t)(((int64_t)((int32_t)update.avg_value_mV - (int32_t)ADC_MIN_CALIBRATED_MV) * (int64_t)output_span_mv) / (int32_t)ADC_SPAN_MV);
            target_mV = adc_mV_to_curved_dac_mV(clamped_value_mV);
        }
        
        /* Update LED */
        if (ws2812_ready && ws2812_buffer != NULL) {
            uint8_t led_green = 0;
            if (update_valid) {
                uint16_t output_min_mv = settings_get_output_min_mv();
                uint16_t output_max_mv = settings_get_output_max_mv();
                uint32_t output_span_mv = (output_max_mv > output_min_mv)
                                              ? ((uint32_t)output_max_mv - (uint32_t)output_min_mv)
                                              : 0;
                uint32_t clamped_led_target = target_mV;
                if (clamped_led_target < output_min_mv) {
                    clamped_led_target = output_min_mv;
                }
                if (clamped_led_target > output_max_mv) {
                    clamped_led_target = output_max_mv;
                }
                if (output_span_mv > 0) {
                    led_green = (uint8_t)(((uint64_t)(clamped_led_target - output_min_mv) * 255u + (output_span_mv / 2u)) / output_span_mv);
                }
            }
            
            uint8_t led_blue = ble_is_connected() ? 8 : 0;
            if (led_green != last_led_green || led_blue != last_led_blue) {
                ws2812_buffer[0] = (CRGB){.r=0, .g=led_green, .b=led_blue};
                ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
                last_led_green = led_green;
                last_led_blue = led_blue;
            }
        }
        
        /* Write DAC */
        uint32_t applied_target_mV = update_valid ? target_mV : 0;
        if (dac_status == DAC80501_OK && applied_target_mV != last_written_target_mV) {
            if (dac80501_write_voltage(&dac80501_dev, applied_target_mV, 0) != DAC80501_OK) {
                ESP_LOGW(TAG, "Failed to write %u mV to DAC", applied_target_mV);
            } else {
                last_written_target_mV = applied_target_mV;
            }
        }
        if (!update.adc_read_valid) {
            ESP_LOGW(TAG, "ADC read failed, driving DAC to zero");
        } else if (!update_valid) {
            ESP_LOGW(TAG, "No valid ADC update, driving DAC to zero");
        }
        
        /* Debug logging */
        now = xTaskGetTickCount();
        int32_t curve_delta_mV = (int32_t)applied_target_mV - unclamped_target_mV;
        bool map_changed = (last_logged_avg_value_mV == UINT32_MAX) ||
                   (u32_abs_diff(update.avg_value_mV, last_logged_avg_value_mV) >= MAP_LOG_CHANGE_MIN_MV) ||
                   (u32_abs_diff(clamped_value_mV, last_logged_clamped_value_mV) >= MAP_LOG_CHANGE_MIN_MV) ||
                   (i32_abs(unclamped_target_mV - last_logged_unclamped_target_mV) >= (int32_t)MAP_LOG_CHANGE_MIN_MV) ||
                   (u32_abs_diff(applied_target_mV, last_logged_applied_target_mV) >= MAP_LOG_CHANGE_MIN_MV) ||
                   (i32_abs(curve_delta_mV - last_logged_curve_delta_mV) >= (int32_t)MAP_LOG_CHANGE_MIN_MV);
        if (map_changed && (last_map_log_tick == 0 || (now - last_map_log_tick) >= map_log_min_period_ticks)) {
            ESP_LOGI(TAG, "Map: s=%u avg=%u in=%u lin=%" PRId32 " out=%u d=%+" PRId32,
                     update.samples_used,
                     update.avg_value_mV,
                     clamped_value_mV,
                     unclamped_target_mV,
                     applied_target_mV,
                     curve_delta_mV);

            last_logged_avg_value_mV = update.avg_value_mV;
            last_logged_clamped_value_mV = clamped_value_mV;
            last_logged_unclamped_target_mV = unclamped_target_mV;
            last_logged_applied_target_mV = applied_target_mV;
            last_logged_curve_delta_mV = curve_delta_mV;
            last_map_log_tick = now;
        }

#if ENABLE_PERF_LOGS
        if (last_perf_log_tick == 0 || (now - last_perf_log_tick) >= perf_log_period_ticks) {
            ESP_LOGI(TAG, "Perf: control_work_max=%lld us control_loop_max=%lld us",
                     (long long)work_us_max,
                     (long long)loop_us_max);
            work_us_max = 0;
            loop_us_max = 0;
            last_perf_log_tick = now;
        }
#endif
        
        ble_notify_latest_adc_mV();

#if ENABLE_PERF_LOGS
        int64_t work_us = esp_timer_get_time() - work_start_us;
        if (work_us > work_us_max) {
            work_us_max = work_us;
        }

        int64_t loop_us = esp_timer_get_time() - loop_start_us;
        if (loop_us > loop_us_max) {
            loop_us_max = loop_us;
        }
#endif
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ADC sampling task: continuous fast sampling on Core 0 */
static void adc_task(void *pvParameters)
{
    (void)pvParameters;
    const char *task_name = pcTaskGetName(NULL);
    ESP_LOGI(task_name, "ADC task started on core %d", xPortGetCoreID());
    
    esp_task_wdt_add(NULL);
    
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    uint32_t ret_num = 0;
    esp_err_t ret;
    
    uint64_t window_sample_sum_mV = 0;
    uint32_t window_sample_count = 0;
    uint32_t last_avg_value_mV = ADC_MIN_CALIBRATED_MV;
    uint32_t filtered_avg_value_mV = ADC_MIN_CALIBRATED_MV;
    bool filter_initialized = false;
    TickType_t last_dac_update_tick = 0;
    const TickType_t dac_update_period_ticks = pdMS_TO_TICKS(DAC_UPDATE_PERIOD_MS);
    TickType_t adc_task_period_ticks = pdMS_TO_TICKS(ADC_TASK_PERIOD_MS);
    if (adc_task_period_ticks == 0) {
        adc_task_period_ticks = 1;
        ESP_LOGW(TAG, "ADC_TASK_PERIOD_MS=%u is below one RTOS tick; clamping cadence to 1 tick", ADC_TASK_PERIOD_MS);
    }
    TickType_t last_adc_wake_tick = xTaskGetTickCount();
#if ENABLE_PERF_LOGS
    int64_t loop_us_max = 0;
    int64_t read_us_max = 0;
    int64_t parse_us_max = 0;
    TickType_t last_perf_log_tick = 0;
    TickType_t perf_log_period_ticks = pdMS_TO_TICKS(DEBUG_LOG_PERIOD_MS);
    if (perf_log_period_ticks == 0) {
        perf_log_period_ticks = 1;
    }
#endif
    
    while (1) {
#if ENABLE_PERF_LOGS
        int64_t loop_start_us = esp_timer_get_time();
#endif
        /* Keep task watchdog happy while doing work in this loop. */
        esp_task_wdt_reset();
        
        uint32_t num_parsed_samples = 0;
        bool adc_read_valid = false;
        bool adc_read_timed_out = false;

#if ENABLE_PERF_LOGS
        int64_t read_start_us = esp_timer_get_time();
#endif
        ret = adc_continuous_read(adc_handle, result, EXAMPLE_READ_LEN, &ret_num, pdMS_TO_TICKS(5));
#if ENABLE_PERF_LOGS
        int64_t read_us = esp_timer_get_time() - read_start_us;
        if (read_us > read_us_max) {
            read_us_max = read_us;
        }
#endif
        if (ret == ESP_OK) {
            adc_read_valid = true;
#if ENABLE_PERF_LOGS
            int64_t parse_start_us = esp_timer_get_time();
#endif
            esp_err_t parse_ret = adc_continuous_parse_data(adc_handle, result, ret_num, s_parsed_data, &num_parsed_samples);
#if ENABLE_PERF_LOGS
            int64_t parse_us = esp_timer_get_time() - parse_start_us;
            if (parse_us > parse_us_max) {
                parse_us_max = parse_us;
            }
#endif
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

        if (adc_read_timed_out && window_sample_count == 0) {
#if ENABLE_PERF_LOGS
            TickType_t perf_now = xTaskGetTickCount();
            if (last_perf_log_tick == 0 || (perf_now - last_perf_log_tick) >= perf_log_period_ticks) {
                ESP_LOGI(TAG, "Perf: adc_read_max=%lld us adc_parse_max=%lld us adc_loop_max=%lld us",
                         (long long)read_us_max,
                         (long long)parse_us_max,
                         (long long)loop_us_max);
                read_us_max = 0;
                parse_us_max = 0;
                loop_us_max = 0;
                last_perf_log_tick = perf_now;
            }
            int64_t loop_us = esp_timer_get_time() - loop_start_us;
            if (loop_us > loop_us_max) {
                loop_us_max = loop_us;
            }
#endif
            vTaskDelayUntil(&last_adc_wake_tick, adc_task_period_ticks);
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if (last_dac_update_tick == 0 || (now - last_dac_update_tick) >= dac_update_period_ticks) {
            uint32_t samples_used = window_sample_count;
            control_update_t update_msg = {0};

                if (window_sample_count > 0) {
                    uint32_t avg_value = (uint32_t)(window_sample_sum_mV / window_sample_count);
                    if (!filter_initialized) {
                        filtered_avg_value_mV = avg_value;
                        filter_initialized = true;
                    } else {
                        /* Simple 1st-order IIR filter: 75% previous + 25% new sample window. */
                        filtered_avg_value_mV = (uint32_t)(((uint64_t)filtered_avg_value_mV * 3u + (uint64_t)avg_value + 2u) / 4u);
                    }
                    last_avg_value_mV = filtered_avg_value_mV;

                    // Send the filtered averaged value over BLE
                    ble_set_latest_adc_mV((uint16_t)last_avg_value_mV);
                }

            /* Queue the update for the control task to handle */
            update_msg.avg_value_mV = last_avg_value_mV;
            update_msg.samples_used = samples_used;
            update_msg.adc_read_valid = adc_read_valid;
            
            /* Keep only latest update; control task should never work stale backlog. */
            xQueueOverwrite(control_queue, &update_msg);

            window_sample_sum_mV = 0;
            window_sample_count = 0;
            last_dac_update_tick = now;
        }

#if ENABLE_PERF_LOGS
        TickType_t perf_now = xTaskGetTickCount();
        if (last_perf_log_tick == 0 || (perf_now - last_perf_log_tick) >= perf_log_period_ticks) {
            ESP_LOGI(TAG, "Perf: adc_read_max=%lld us adc_parse_max=%lld us adc_loop_max=%lld us",
                     (long long)read_us_max,
                     (long long)parse_us_max,
                     (long long)loop_us_max);
            read_us_max = 0;
            parse_us_max = 0;
            loop_us_max = 0;
            last_perf_log_tick = perf_now;
        }

        int64_t loop_us = esp_timer_get_time() - loop_start_us;
        if (loop_us > loop_us_max) {
            loop_us_max = loop_us;
        }
#endif

        vTaskDelayUntil(&last_adc_wake_tick, adc_task_period_ticks);
    }
}

void app_main(void) {

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
    
    /* Store handle globally so adc_task can access it */
    adc_handle = handle;

    sys_init();

    ESP_LOGI(TAG, "Output range: min=%u mV max=%u mV",
             settings_get_output_min_mv(),
             settings_get_output_max_mv());
    
    /* Create queue for passing control updates from ADC task to control task */
    control_queue = xQueueCreate(1, sizeof(control_update_t));
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control queue");
        return;
    }
    
    /* Start ADC task on core 0 with high priority */
    xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 7, NULL, 0);
    
    /* Start control task on core 1 */
    xTaskCreatePinnedToCore(control_task, "control_task", 8192, NULL, 5, NULL, 1);
    
    /* app_main returns and becomes idle task - adc_task and control_task handle watchdog */
}