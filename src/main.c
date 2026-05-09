// System includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"

// Project includes
#include "system.h"
#include "esp_ws28xx.h"

static const char *TAG = "Footsie";

CRGB *ws2812_buffer;

static void sys_init(void) {
    // Delay to wait for uart connection so nothing is missed.
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    char *task_name = pcTaskGetName(NULL);
    ESP_LOGI(task_name, "Initialisation Started.");

    esp_log_level_set("I2C", ESP_LOG_DEBUG);

    // Perform I2C scan to detect devices on the bus
    // esp_err_t scan_result = i2c_scan();
    // if (scan_result != ESP_OK) {
    //     ESP_LOGE(TAG, "I2C scan failed: %s", esp_err_to_name(scan_result));
    //     //return scan_result;
    // }

    // I2C bus configuration
    // i2c_master_bus_config_t i2c_master_config = {
    //     .clk_source = I2C_CLK_SRC_DEFAULT,
    //     .i2c_port = I2C_PORT_NUM,
    //     .scl_io_num = PIN_I2C_SCL,
    //     .sda_io_num = PIN_I2C_SDA,
    //     .glitch_ignore_cnt = 7,
    //     .flags.enable_internal_pullup = false,
    // };
    // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &g_i2c_bus_handle));

    gpio_reset_pin(PIN_ADC_IN);
    gpio_set_direction(PIN_ADC_IN, GPIO_MODE_INPUT);

    // WS2812B RGB LED driver initialisation.
    ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_init(PIN_RGB, WS2812B, 1, &ws2812_buffer));

    // Set LED to green as init complete confirmation
    ws2812_buffer[0] = (CRGB){.r=0, .g=1, .b=0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());

    ESP_LOGI(task_name, "Initialisation Complete.");
}

void app_main(void) {

    sys_init();

    while(1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}