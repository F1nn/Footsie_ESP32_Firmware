#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize BLE stack and services. This will call `nvs_flash_init()` internally. */
void ble_init(void);

/* Update the latest ADC millivolt value that GATT can read/notify. */
void ble_set_latest_adc_mV(uint16_t mv);

/* If connected and notifications enabled, notify the peer with the latest ADC millivolt value. */
void ble_notify_latest_adc_mV(void);

/* Periodic BLE housekeeping, including stale-connection recovery. */
void ble_poll(void);

/* Query connection state */
bool ble_is_connected(void);

#endif /* BLE_MODULE_H */
