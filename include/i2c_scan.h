
#ifndef I2C_SCAN_H
#define I2C_SCAN_H

// Include necessary libraries
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "system.h"

esp_err_t i2c_scan(void);

#endif // I2C_SCAN_H