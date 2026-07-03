#include "ble.h"
#include "system.h"
#include "settings.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"

static const char *TAG = "ble_mod";

static volatile bool s_ble_connected = false;
static volatile bool s_ble_raw_adc_notify_enabled = false;
static uint8_t s_ble_addr_type = 0;
static uint16_t s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_ble_raw_adc_val_handle = 0;
static uint16_t s_latest_adc_mv = 0;

// Enum to identify which characteristic owns a descriptor
typedef enum {
    BLE_DSC_CHR_GAMMA = 1,
    BLE_DSC_CHR_ADC_RAW = 2,
    BLE_DSC_CHR_SLIDER = 3,
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

static const ble_uuid128_t g_ble_slider_cal_chr_uuid =
    BLE_UUID128_INIT(0x50, 0x1e, 0x5b, 0x9c, 0x76, 0x4b, 0x4b, 0x18,
                     0x9d, 0x87, 0x79, 0x6d, 0x03, 0x00, 0x8c, 0x4a);

static const ble_uuid16_t g_ble_user_desc_uuid = BLE_UUID16_INIT(0x2901);

static void ble_restart_advertising(void);

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
        desc_str = "ADC mV";
    } else if (chr_id == BLE_DSC_CHR_SLIDER) {
        desc_str = "ADC cal (min,max mV)";
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
            uint16_t gamma_x100 = settings_get_gamma_x100();
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

            if (settings_set_gamma_x100(gamma_x100) != ESP_OK) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            ESP_LOGI(TAG, "BLE gamma updated: %u.%02u", gamma_x100 / 100u, gamma_x100 % 100u);
            return 0;
        }

        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &g_ble_adc_raw_chr_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint16_t mv = s_latest_adc_mv;
            return os_mbuf_append(ctxt->om, &mv, sizeof(mv)) == 0
                       ? 0
                       : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &g_ble_slider_cal_chr_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint16_t min_mv = settings_get_adc_min_mv();
            uint16_t max_mv = settings_get_adc_max_mv();
            uint16_t buf[2] = { min_mv, max_mv };
            return os_mbuf_append(ctxt->om, buf, sizeof(buf)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t vals[2] = {0,0};
            uint16_t bytes_copied = 0;

            if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(vals)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (ble_hs_mbuf_to_flat(ctxt->om,
                                    vals,
                                    sizeof(vals),
                                    &bytes_copied) != 0 ||
                bytes_copied != sizeof(vals)) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            uint16_t min_mv = vals[0];
            uint16_t max_mv = vals[1];

            if (min_mv >= max_mv || max_mv > ADC_VREF_MV) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (settings_set_adc_calibrated_mv(min_mv, max_mv) != ESP_OK) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            ESP_LOGI(TAG, "BLE ADC calibration updated: min=%u mV max=%u mV", min_mv, max_mv);
            return 0;
        }

        return BLE_ATT_ERR_UNLIKELY;
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
                .uuid = &g_ble_slider_cal_chr_uuid.u,
                .access_cb = ble_gatt_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = &g_ble_user_desc_uuid.u,
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = ble_gatt_dsc_access,
                        .arg = (void *)(uintptr_t)BLE_DSC_CHR_SLIDER,
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
            ble_restart_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        s_ble_connected = false;
        s_ble_raw_adc_notify_enabled = false;
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_restart_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_ble_raw_adc_val_handle) {
            s_ble_raw_adc_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG,
                     "ADC notify subscription changed: handle=%u enabled=%d",
                     event->subscribe.conn_handle,
                     s_ble_raw_adc_notify_enabled);

            if (!s_ble_raw_adc_notify_enabled && s_ble_connected &&
                s_ble_conn_handle == event->subscribe.conn_handle) {
                ESP_LOGI(TAG, "ADC notify disabled, terminating BLE connection");
                int rc = ble_gap_terminate(event->subscribe.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                if (rc != 0 && rc != BLE_HS_ENOTCONN) {
                    ESP_LOGW(TAG, "ble_gap_terminate failed, rc=%d", rc);
                }
            }
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_restart_advertising();
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

static void ble_restart_advertising(void)
{
    ble_start_advertising();
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

void ble_set_latest_adc_mV(uint16_t mv)
{
    s_latest_adc_mv = mv;
}

void ble_notify_latest_adc_mV(void)
{
    if (!s_ble_connected || !s_ble_raw_adc_notify_enabled ||
        s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        s_ble_raw_adc_val_handle == 0) {
        return;
    }

    uint16_t mv = s_latest_adc_mv;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&mv, sizeof(mv));
    if (om == NULL) {
        return;
    }

    int rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_raw_adc_val_handle, om);
    if (rc != 0 && rc != BLE_HS_ENOTCONN && rc != BLE_HS_EBUSY && rc != BLE_HS_ENOMEM) {
        ESP_LOGW(TAG, "ble_gatts_notify_custom failed, rc=%d", rc);
    }
}

void ble_poll(void)
{
    if (!s_ble_connected || s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    struct ble_gap_conn_desc conn_desc;
    int rc = ble_gap_conn_find(s_ble_conn_handle, &conn_desc);
    if (rc == 0) {
        return;
    }

    ESP_LOGW(TAG, "Stale BLE connection handle %u detected, rc=%d", s_ble_conn_handle, rc);
    s_ble_connected = false;
    s_ble_raw_adc_notify_enabled = false;
    s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ble_restart_advertising();
}

bool ble_is_connected(void)
{
    return s_ble_connected && s_ble_raw_adc_notify_enabled;
}

void ble_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(nimble_port_init());

    /* Reduce verbose NimBLE stack logs (e.g. frequent GATT notify/info lines) */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(BLE_DEVICE_NAME));

    ESP_ERROR_CHECK(ble_gatts_count_cfg(g_ble_gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(g_ble_gatt_svcs));

    nimble_port_freertos_init(ble_host_task);
}
