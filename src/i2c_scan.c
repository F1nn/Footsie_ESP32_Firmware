#include "i2c_scan.h"

#define I2C_SCAN_TIMEOUT_MS 100

static const char *TAG = "I2C_SCAN";

esp_err_t i2c_scan(void) {
    ESP_LOGI(TAG, "Starting I2C bus scan...");
    esp_err_t ret;
    bool found_device = false;
    i2c_master_bus_handle_t temp_bus_handle = NULL;

    // Initialize temporary I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    
    ret = i2c_new_master_bus(&bus_cfg, &temp_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Scan all possible addresses
    for (uint8_t address = 1; address < 127; address++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 100000,
        };

        i2c_master_dev_handle_t temp_dev_handle;
        ret = i2c_master_bus_add_device(temp_bus_handle, &dev_cfg, &temp_dev_handle);
        
        if (ret == ESP_OK) {
            ret = i2c_master_probe(temp_bus_handle, address, pdMS_TO_TICKS(I2C_SCAN_TIMEOUT_MS));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Device found at address: 0x%02X", address);
                found_device = true;
            }
            i2c_master_bus_rm_device(temp_dev_handle);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!found_device) {
        ESP_LOGW(TAG, "No I2C devices found");
    }

    // Cleanup
    if (temp_bus_handle) {
        i2c_del_master_bus(temp_bus_handle);
    }
    
    return ESP_OK;
}
