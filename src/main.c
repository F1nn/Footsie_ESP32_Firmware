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
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "system.h"
#include "esp_ws28xx.h"
#include "dac80501.h"

#define EXAMPLE_READ_LEN                    64 // 16 samples, each sample has 4 bytes (see SOC_ADC_DIGI_RESULT_BYTES)
#define MAX_PARSED_SAMPLES                  (EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES)
/* TODO: move calibration and scale constants to Kconfig/menuconfig or NVS */
/* ADC reference (in mV) for converting ADC code to millivolts */
#define ADC_VREF_MV                         3300
/* DAC full-scale output target in mV */
#define DAC_FULL_SCALE_MV                   5000
/* Maximum DAC update rate */
#define DAC_UPDATE_PERIOD_MS                20
/* Debug print interval for mapping telemetry */
#define DEBUG_LOG_PERIOD_MS                 1000
/* Output curve exponent: >1.0 gives finer control at the low end. */
#define OUTPUT_CURVE_GAMMA                  2.2f
#define OUTPUT_CURVE_GAMMA_MIN_X100         50u
#define OUTPUT_CURVE_GAMMA_MAX_X100         500u
#define BLE_DEVICE_NAME                     "Footsie"
#define NVS_NAMESPACE                       "footsie"
#define NVS_KEY_GAMMA                       "gamma_x100"
/* ADC calibrated range (mV) - adjust these to match your potentiometer's actual measurement range */
/* Set to the calibrated ADC reading at the pot's minimum position */
#define ADC_MIN_CALIBRATED_MV               139
/* Set to the calibrated ADC reading at the pot's maximum position */
#define ADC_MAX_CALIBRATED_MV               3181
#define ADC_SPAN_MV                         (ADC_MAX_CALIBRATED_MV - ADC_MIN_CALIBRATED_MV)

static adc_channel_t channel[1] = {ADC_CHANNEL_2}; // Channel 2 maps to GPIO3

static const char *TAG = "Footsie";

CRGB *ws2812_buffer;
static bool ws2812_ready = false;
dac80501_device_t dac80501_dev;
i2c_master_bus_handle_t i2c_bus_handle = NULL;
static dac80501_status_t dac_status = DAC80501_ERR_NOT_INITIALIZED;
static adc_cali_handle_t adc_cali_handle = NULL;

static adc_continuous_data_t s_parsed_data[MAX_PARSED_SAMPLES];
static volatile float s_output_curve_gamma = OUTPUT_CURVE_GAMMA;
static volatile uint16_t s_latest_adc_raw = 0;

static volatile bool s_ble_connected = false;
static volatile bool s_ble_raw_adc_notify_enabled = false;
static uint8_t s_ble_addr_type = 0;
static uint16_t s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_ble_raw_adc_val_handle = 0;

// Enum to identify which characteristic owns a descriptor
typedef enum {
    BLE_DSC_CHR_GAMMA = 1,
    BLE_DSC_CHR_ADC_RAW = 2,
} ble_dsc_chr_id_t;

static const ble_uuid128_t g_ble_svc_uuid =
    BLE_UUID128_INIT(0x50, 0x1e, 0x5b, 0x9c, 0x76, 0x4b, 0x4b, 0x18,
                     0x9d, 0x87, 0x79, 0x6d, 0x00, 0x00, 0x8c, 0x4a);
static const ble_uuid128_t g_ble_gamma_chr_uuid =
    BLE_UUID128_INIT(0x50, 0x1e, 0x5b, 0x9c, 0x76, 0x4b, 0x4b, 0x18,
                     0x9d, 0x87, 0x79, 0x6d, 0x01, 0x00, 0x8c, 0x4a);
static const ble_uuid128_t g_ble_adc_raw_chr_uuid =
    BLE_UUID128_INIT(0x50, 0x1e, 0x5b, 0x9c, 0x76, 0x4b, 0x4b, 0x18,
                     0x9d, 0x87, 0x79, 0x6d, 0x02, 0x00, 0x8c, 0x4a);

static const ble_uuid16_t g_ble_user_desc_uuid =
    BLE_UUID16_INIT(0x2901);

static void ble_start_advertising(void);
static void ble_notify_latest_adc_raw(void);
static void gamma_load_from_nvs(void);
static void gamma_save_to_nvs(uint16_t gamma_x100);

static int ble_gatt_dsc_access(uint16_t conn_handle,
                               uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt,
                               void *arg)
{
    (void)conn_handle;
    (void)attr_handle;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    ble_dsc_chr_id_t chr_id = (ble_dsc_chr_id_t)(uintptr_t)arg;
    const char *desc_str = NULL;

    if (chr_id == BLE_DSC_CHR_GAMMA) {
        desc_str = "Gamma x100";
    } else if (chr_id == BLE_DSC_CHR_ADC_RAW) {
        desc_str = "Raw ADC";
    } else {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return os_mbuf_append(ctxt->om, (const void *)desc_str, strlen(desc_str)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int ble_gatt_chr_access(uint16_t conn_handle,
                               uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt,
                               void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ble_uuid_cmp(ctxt->chr->uuid, &g_ble_gamma_chr_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint16_t gamma_x100 = (uint16_t)((s_output_curve_gamma * 100.0f) + 0.5f);
            return os_mbuf_append(ctxt->om, &gamma_x100, sizeof(gamma_x100)) == 0
                       ? 0
                       : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t gamma_x100 = 0;
            uint16_t bytes_copied = 0;

            if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(gamma_x100)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (ble_hs_mbuf_to_flat(ctxt->om,
                                    &gamma_x100,
                                    sizeof(gamma_x100),
                                    &bytes_copied) != 0 ||
                bytes_copied != sizeof(gamma_x100)) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            if (gamma_x100 < OUTPUT_CURVE_GAMMA_MIN_X100 || gamma_x100 > OUTPUT_CURVE_GAMMA_MAX_X100) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            s_output_curve_gamma = (float)gamma_x100 / 100.0f;
            ESP_LOGI(TAG, "BLE gamma updated: %u.%02u", gamma_x100 / 100u, gamma_x100 % 100u);
            gamma_save_to_nvs(gamma_x100);
            return 0;
        }

        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &g_ble_adc_raw_chr_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint16_t raw = s_latest_adc_raw;
            return os_mbuf_append(ctxt->om, &raw, sizeof(raw)) == 0
                       ? 0
                       : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def g_ble_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &g_ble_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &g_ble_gamma_chr_uuid.u,
                .access_cb = ble_gatt_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = &g_ble_user_desc_uuid.u,
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = ble_gatt_dsc_access,
                        .arg = (void *)(uintptr_t)BLE_DSC_CHR_GAMMA,
                    },
                    {0},
                },
            },
            {
                .uuid = &g_ble_adc_raw_chr_uuid.u,
                .access_cb = ble_gatt_chr_access,
                .val_handle = &s_ble_raw_adc_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = &g_ble_user_desc_uuid.u,
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = ble_gatt_dsc_access,
                        .arg = (void *)(uintptr_t)BLE_DSC_CHR_ADC_RAW,
                    },
                    {0},
                },
            },
            {0},
        },
    },
    {0},
};

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset, reason=%d", reason);
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_ble_connected = true;
            s_ble_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected, handle=%u", s_ble_conn_handle);
        } else {
            s_ble_connected = false;
            s_ble_raw_adc_notify_enabled = false;
            s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "BLE connect failed, status=%d", event->connect.status);
            ble_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        s_ble_connected = false;
        s_ble_raw_adc_notify_enabled = false;
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_ble_raw_adc_val_handle) {
            s_ble_raw_adc_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG,
                     "ADC notify subscription changed: handle=%u enabled=%d",
                     event->subscribe.conn_handle,
                     s_ble_raw_adc_notify_enabled);
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_start_advertising();
        return 0;

    default:
        return 0;
    }
}

static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields scan_rsp_fields;
    struct ble_gap_adv_params adv_params;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.uuids128 = (ble_uuid128_t *)&g_ble_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed, rc=%d", rc);
        return;
    }

    memset(&scan_rsp_fields, 0, sizeof(scan_rsp_fields));
    scan_rsp_fields.name = (const uint8_t *)BLE_DEVICE_NAME;
    scan_rsp_fields.name_len = strlen(BLE_DEVICE_NAME);
    scan_rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed, rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_ble_addr_type,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           ble_gap_event_cb,
                           NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed, rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE advertising started");
    }
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed, rc=%d", rc);
        return;
    }

    ble_start_advertising();
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_notify_latest_adc_raw(void)
{
    if (!s_ble_connected || !s_ble_raw_adc_notify_enabled ||
        s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        s_ble_raw_adc_val_handle == 0) {
        return;
    }

    uint16_t raw = s_latest_adc_raw;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&raw, sizeof(raw));
    if (om == NULL) {
        return;
    }

    int rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_raw_adc_val_handle, om);
    if (rc != 0 && rc != BLE_HS_ENOTCONN && rc != BLE_HS_EBUSY && rc != BLE_HS_ENOMEM) {
        ESP_LOGW(TAG, "ble_gatts_notify_custom failed, rc=%d", rc);
    }
}

static void gamma_load_from_nvs(void)
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
}

static void gamma_save_to_nvs(uint16_t gamma_x100)
{
    if (gamma_x100 < OUTPUT_CURVE_GAMMA_MIN_X100 || gamma_x100 > OUTPUT_CURVE_GAMMA_MAX_X100) {
        ESP_LOGE(TAG, "Cannot save gamma %u: out of range [%u, %u]", gamma_x100, OUTPUT_CURVE_GAMMA_MIN_X100, OUTPUT_CURVE_GAMMA_MAX_X100);
        return;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing (%s)", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u16(nvs_handle, NVS_KEY_GAMMA, gamma_x100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write gamma to NVS (%s)", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit gamma to NVS (%s)", esp_err_to_name(err));
        return;
    }

    ESP_LOGD(TAG, "Gamma saved to NVS: %u.%02u", gamma_x100 / 100u, gamma_x100 % 100u);
}

static void ble_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(BLE_DEVICE_NAME));

    ESP_ERROR_CHECK(ble_gatts_count_cfg(g_ble_gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(g_ble_gatt_svcs));

    nimble_port_freertos_init(ble_host_task);
}

static uint32_t adc_mV_to_curved_dac_mV(uint32_t adc_mV)
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

    float curved = powf(normalized, s_output_curve_gamma);
    uint32_t output_mV = (uint32_t)((curved * (float)DAC_FULL_SCALE_MV) + 0.5f);
    if (output_mV > DAC_FULL_SCALE_MV) {
        output_mV = DAC_FULL_SCALE_MV;
    }

    return output_mV;
}

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

    gamma_load_from_nvs();
    ESP_LOGI(TAG, "Output curve gamma: %0.3f", (double)s_output_curve_gamma);

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
        bool adc_read_valid = false;

        ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
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
            bool update_valid = false;
            float last_normalized_value = 0.0f;
            float last_curved_value = 0.0f;

                if (window_sample_count > 0) {
                    uint32_t avg_value = (uint32_t)(window_sample_sum_mV / window_sample_count);
                    last_avg_value_mV = avg_value;

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
                        last_curved_value = powf(last_normalized_value, s_output_curve_gamma);
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

                uint8_t led_blue = s_ble_connected ? 8 : 0;
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

            ble_notify_latest_adc_raw();

            window_sample_sum_mV = 0;
            window_sample_count = 0;
            last_dac_update_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

}