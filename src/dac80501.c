#include "dac80501.h"
#include "esp_log.h"

static const char *TAG = "DAC80501";

/**
 * @brief Write 16-bit value to DAC register
 */
static dac80501_status_t dac80501_write_register(dac80501_device_t *device, uint8_t reg_addr, uint16_t value)
{
    if (!device || !device->dev_handle) {
        return DAC80501_ERR_INVALID_PARAM;
    }

    uint8_t data[3] = {
        reg_addr,
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF)
    };

    esp_err_t ret = i2c_master_transmit(device->dev_handle, data, sizeof(data), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write register 0x%02x failed: 0x%02x", reg_addr, ret);
        return DAC80501_ERR_WRITE_FAILED;
    }

    return DAC80501_OK;
}

/**
 * @brief Read 16-bit value from DAC register
 */
static dac80501_status_t dac80501_read_register(dac80501_device_t *device, uint8_t reg_addr, uint16_t *value)
{
    if (!device || !device->dev_handle || !value) {
        return DAC80501_ERR_INVALID_PARAM;
    }

    uint8_t reg = reg_addr;
    uint8_t data[2] = {0};

    esp_err_t ret = i2c_master_transmit_receive(device->dev_handle, &reg, 1, data, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read register 0x%02x failed: 0x%02x", reg_addr, ret);
        return DAC80501_ERR_READ_FAILED;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];
    return DAC80501_OK;
}

static void dac80501_log_register(dac80501_device_t *device, uint8_t reg_addr)
{
    uint16_t value = 0;

    if (dac80501_read_register(device, reg_addr, &value) == DAC80501_OK) {
        ESP_LOGI(TAG, "Register 0x%02x = 0x%04x", reg_addr, value);
    } else {
        ESP_LOGW(TAG, "Register 0x%02x read failed", reg_addr);
    }
}

dac80501_status_t dac80501_init(dac80501_device_t *device, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr)
{
    if (!device) {
        return DAC80501_ERR_INVALID_PARAM;
    }

    device->dev_handle = NULL;

    device->i2c_addr = i2c_addr;
    device->bus_handle = bus_handle;
    device->vref_mV = DAC80501_DEFAULT_VREF_MV;
    /* defaults per datasheet: BUF-GAIN = 1 (2x vref), REF-DIV = 0 (no divider) */
    device->buf_gain = 1;
    device->ref_div = 0;

    /* Create I2C device handle */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &device->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: 0x%02x", ret);
        return DAC80501_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "I2C device added at address 0x%02x", i2c_addr);

    ESP_LOGI(TAG, "Register dump on init:");
    dac80501_log_register(device, DAC80501_REG_NOOP);
    dac80501_log_register(device, DAC80501_REG_DEVICEID);
    dac80501_log_register(device, DAC80501_REG_SYNC);
    dac80501_log_register(device, DAC80501_REG_CONFIG);
    dac80501_log_register(device, DAC80501_REG_GAIN);
    dac80501_log_register(device, DAC80501_REG_TRIGGER);
    dac80501_log_register(device, DAC80501_REG_STATUS);
    dac80501_log_register(device, DAC80501_REG_DAC_DATA);

    /* Verify device identity using the DEVID register */
    uint16_t device_id = 0;
    dac80501_status_t status = dac80501_get_device_id(device, &device_id);
    if (status != DAC80501_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        i2c_master_bus_rm_device(device->dev_handle);
        device->dev_handle = NULL;
        return status;
    }

    ESP_LOGI(TAG, "Device ID: 0x%04x", device_id);

    if (device_id != DAC80501_DEVICE_ID) {
        ESP_LOGE(TAG, "Device ID mismatch: got 0x%04x, expected 0x%04x",
                 device_id, DAC80501_DEVICE_ID);
        i2c_master_bus_rm_device(device->dev_handle);
        device->dev_handle = NULL;
        return DAC80501_ERR_DEVICE_ID;
    }

    /* Configure hardware registers for default buffering/divider so output
       maps to 0-5V by default when VDD=5V and internal ref=2.5V.
       REF-DIV is in GAIN, not CONFIG. */
    if (dac80501_set_buf_gain(device, device->buf_gain) != DAC80501_OK) {
        ESP_LOGW(TAG, "Failed to set BUF-GAIN to %u", device->buf_gain);
    }

    if (dac80501_set_ref_div(device, device->ref_div) != DAC80501_OK) {
        ESP_LOGW(TAG, "Failed to set REF-DIV to %u", device->ref_div);
    }

    ESP_LOGI(TAG, "DAC80501 device initialized at address 0x%02x", i2c_addr);
    return DAC80501_OK;
}

dac80501_status_t dac80501_deinit(dac80501_device_t *device)
{
    if (!device) {
        return DAC80501_ERR_INVALID_PARAM;
    }

    if (!device->dev_handle) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }

    esp_err_t ret = i2c_master_bus_rm_device(device->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: 0x%02x", ret);
        return DAC80501_ERR_WRITE_FAILED;
    }

    device->dev_handle = NULL;

    ESP_LOGI(TAG, "DAC80501 deinitialized");
    return DAC80501_OK;
}

dac80501_status_t dac80501_write_dac(dac80501_device_t *device, uint16_t value)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }

    return dac80501_write_register(device, DAC80501_REG_DAC_DATA, value);
}

dac80501_status_t dac80501_write_voltage(dac80501_device_t *device, uint32_t mV, uint32_t vref_mV)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }

    /* allow caller to pass vref_mV == 0 to use device default adjusted by buf_gain/ref_div */
    if (vref_mV == 0) {
        uint64_t eff = device->vref_mV;

        /* BUF-GAIN doubles the reference when enabled. */
        if (device->buf_gain) {
            eff *= 2;
        }

        /* REF-DIV is a single bit in the GAIN register. When set, it divides
           the reference by 2; when cleared, the reference is unchanged. */
        if (device->ref_div != 0) {
            eff /= 2;
        }

        vref_mV = (uint32_t)eff;
    }

    /* Clamp input voltage to 0..vref_mV */
    if (mV == 0) {
        return dac80501_write_dac(device, 0x0000);
    }

    uint32_t code;
    if (mV >= vref_mV) {
        code = 0xFFFF;
    } else {
        /* scale to 0..65535, perform 64-bit ops to be safe */
        code = (uint32_t)(((uint64_t)mV * 0xFFFF + (vref_mV / 2)) / vref_mV);
    }

    return dac80501_write_dac(device, (uint16_t)(code & 0xFFFF));
}

dac80501_status_t dac80501_read_dac(dac80501_device_t *device, uint16_t *value)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }

    return dac80501_read_register(device, DAC80501_REG_DAC_DATA, value);
}

dac80501_status_t dac80501_get_status(dac80501_device_t *device, uint16_t *status)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }

    return dac80501_read_register(device, DAC80501_REG_STATUS, status);
}

dac80501_status_t dac80501_get_device_id(dac80501_device_t *device, uint16_t *device_id)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }

    return dac80501_read_register(device, DAC80501_REG_DEVICEID, device_id);
}

dac80501_status_t dac80501_set_vref(dac80501_device_t *device, uint32_t vref_mV)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }
    if (vref_mV == 0) {
        return DAC80501_ERR_INVALID_PARAM;
    }
    device->vref_mV = vref_mV;
    return DAC80501_OK;
}

dac80501_status_t dac80501_set_buf_gain(dac80501_device_t *device, uint8_t enable)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }
    /* Update device struct */
    device->buf_gain = enable ? 1 : 0;
    /* Write to GAIN register. Preserve REF-DIV bit while updating BUF-GAIN. */
    uint16_t gain = 0;
    dac80501_status_t status = dac80501_read_register(device, DAC80501_REG_GAIN, &gain);
    if (status != DAC80501_OK) {
        return status;
    }

    gain &= (uint16_t)~0x0001;
    gain |= (uint16_t)(device->buf_gain ? 0x0001 : 0x0000);

    return dac80501_write_register(device, DAC80501_REG_GAIN, gain);
}

dac80501_status_t dac80501_set_ref_div(dac80501_device_t *device, uint8_t ref_div)
{
    if (!device) {
        return DAC80501_ERR_NOT_INITIALIZED;
    }
    device->ref_div = ref_div;
    /* REF-DIV is in the GAIN register, bit 8. Preserve BUF-GAIN while updating REF-DIV. */
    uint16_t gain = 0;
    dac80501_status_t status = dac80501_read_register(device, DAC80501_REG_GAIN, &gain);
    if (status != DAC80501_OK) {
        return status;
    }

    gain &= (uint16_t)~0x0100;
    gain |= (uint16_t)((ref_div ? 1u : 0u) << 8);

    return dac80501_write_register(device, DAC80501_REG_GAIN, gain);
}
