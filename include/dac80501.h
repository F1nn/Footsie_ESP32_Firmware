#ifndef DAC80501_H
#define DAC80501_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DAC80501 I2C driver
 * 
 * 16-bit Precision Digital-to-Analog Converter (DAC80501)
 * I2C interface for ESP32 applications
 */

/* I2C Address Configuration */
#define DAC80501_I2C_ADDR_BASE          0x48  /* Base I2C address (A0=0, A1=0) */
#define DAC80501_I2C_ADDR_A0            0x01  /* Address pin A0 offset */
#define DAC80501_I2C_ADDR_A1            0x02  /* Address pin A1 offset */

/* Register Addresses */
#define DAC80501_REG_NOOP               0x00  /* No operation register */
#define DAC80501_REG_DEVICEID           0x01  /* Device ID register */
#define DAC80501_REG_SYNC               0x02  /* Synchronization register */
#define DAC80501_REG_CONFIG             0x03  /* Device configuration register */
#define DAC80501_REG_GAIN               0x04  /* Gain configuration register */
#define DAC80501_REG_TRIGGER            0x05  /* DAC trigger register */
#define DAC80501_REG_STATUS             0x07  /* Device status register */
#define DAC80501_REG_DAC_DATA           0x08  /* DAC output data register */

/* Device ID */
#define DAC80501_DEVICE_ID              0x0115

/* Default reference voltage in millivolts (device internal reference). */
#define DAC80501_DEFAULT_VREF_MV        2500

/* Error Codes */
typedef enum {
    DAC80501_OK = 0,                           /* Operation successful */
    DAC80501_ERR_INVALID_ADDR = -1,            /* Invalid I2C address */
    DAC80501_ERR_NOT_FOUND = -2,               /* Device not found */
    DAC80501_ERR_WRITE_FAILED = -3,            /* Write operation failed */
    DAC80501_ERR_READ_FAILED = -4,             /* Read operation failed */
    DAC80501_ERR_INVALID_PARAM = -5,           /* Invalid parameter */
    DAC80501_ERR_DEVICE_ID = -6,               /* Device ID mismatch */
    DAC80501_ERR_NOT_INITIALIZED = -7,         /* Driver not initialized */
} dac80501_status_t;

/* Device Configuration Structure */
typedef struct {
    uint8_t i2c_addr;                          /* I2C address (0x48-0x4F) */
    i2c_master_bus_handle_t bus_handle;        /* I2C bus handle */
    i2c_master_dev_handle_t dev_handle;        /* I2C device handle */
    uint32_t vref_mV;                          /* Internal reference in mV used for voltage mapping */
    uint8_t buf_gain;                          /* Buffer gain enable (0=1x, 1=2x) */
    uint8_t ref_div;                           /* Reference divider bit (0 = no divide, 1 = divide by 2) */
} dac80501_device_t;

/**
 * @brief Initialize DAC80501 device on I2C bus
 * 
 * @param device Pointer to device structure
 * @param bus_handle I2C bus handle
 * @param i2c_addr I2C address of device (default: 0x48)
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_init(dac80501_device_t *device, 
                                 i2c_master_bus_handle_t bus_handle,
                                 uint8_t i2c_addr);

/**
 * @brief Deinitialize DAC80501 device
 * 
 * @param device Pointer to device structure
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_deinit(dac80501_device_t *device);

/**
 * @brief Write DAC output value
 * 
 * @param device Pointer to device structure
 * @param value 16-bit DAC value (0x0000 to 0xFFFF)
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_write_dac(dac80501_device_t *device, uint16_t value);

/**
 * @brief Read current DAC output value
 * 
 * @param device Pointer to device structure
 * @param value Pointer to store read value
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_read_dac(dac80501_device_t *device, uint16_t *value);

/**
 * @brief Write a target voltage (in millivolts) to the DAC.
 *
 * This helper converts a desired voltage into the 16-bit DAC code
 * using the provided reference voltage and writes it to the device.
 *
 * @param device Pointer to device structure
 * @param mV Desired voltage in millivolts (0..vref_mV)
 * @param vref_mV Reference voltage in millivolts used by the DAC (e.g. 3300)
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_write_voltage(dac80501_device_t *device, uint32_t mV, uint32_t vref_mV);

/**
 * @brief Get device status
 * 
 * @param device Pointer to device structure
 * @param status Pointer to store status value
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_get_status(dac80501_device_t *device, uint16_t *status);

/**
 * @brief Get device ID
 * 
 * @param device Pointer to device structure
 * @param device_id Pointer to store device ID
 * @return DAC80501_OK on success, error code otherwise
 */
dac80501_status_t dac80501_get_device_id(dac80501_device_t *device, uint16_t *device_id);

/**
 * @brief Set device full-scale Vref in millivolts stored in the device struct.
 *
 * This value is used by dac80501_write_voltage when the vref_mV parameter
 * passed to that function is 0.
 */
dac80501_status_t dac80501_set_vref(dac80501_device_t *device, uint32_t vref_mV);

/**
 * @brief Enable/disable output buffer gain (BUF-GAIN). When enabled the
 *        chip's output buffer doubles the internal reference (2x).
 *
 * This writes the appropriate GAIN register and updates the device struct.
 */
dac80501_status_t dac80501_set_buf_gain(dac80501_device_t *device, uint8_t enable);

/**
 * @brief Set reference divider (REF-DIV) field in the GAIN register.
 *
 * A value of 0 means no divider (default). Higher values divide the
 * internal reference as documented in the datasheet.
 */
dac80501_status_t dac80501_set_ref_div(dac80501_device_t *device, uint8_t ref_div);

#ifdef __cplusplus
}
#endif

#endif /* DAC80501_H */
