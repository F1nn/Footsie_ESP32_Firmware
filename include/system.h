#ifndef SYSTEM_H
#define SYSTEM_H

#include "driver/i2c_master.h"

// Pins
#define PIN_I2C_SDA     1
#define PIN_I2C_SCL     2
#define PIN_ADC_IN      3
#define PIN_RGB         21
#define PIN_UART0_TX    43
#define PIN_UART0_RX    44

// Global Variables

#define I2C_PORT_NUM I2C_NUM_0

/* Application configuration and defaults */
#define EXAMPLE_READ_LEN                    64 /* 16 samples, each sample has 4 bytes */
#define MAX_PARSED_SAMPLES                  (EXAMPLE_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES)

/* ADC reference (in mV) for converting ADC code to millivolts */
#define ADC_VREF_MV                         3300
/* DAC full-scale output target in mV */
#define DAC_FULL_SCALE_MV                   5000
/* Maximum DAC update rate */
#define DAC_UPDATE_PERIOD_MS                20
/* ADC task cadence to leave CPU time for IDLE0 */
#define ADC_TASK_PERIOD_MS                  10
/* BLE housekeeping cadence */
#define BLE_POLL_PERIOD_MS                  1000
/* Debug print interval for mapping telemetry */
#define DEBUG_LOG_PERIOD_MS                 1000
/* Map log policy: log only on value changes, and never faster than this period. */
#define MAP_LOG_MIN_PERIOD_MS               500
/* Require at least this change before map logs emit again. */
#define MAP_LOG_CHANGE_MIN_MV               4
/* Enable or disable performance instrumentation logs at compile time. */
#define ENABLE_PERF_LOGS                    0
/* Output curve exponent: >1.0 gives finer control at the low end. */
#define OUTPUT_CURVE_GAMMA                  2.2f
#define OUTPUT_CURVE_GAMMA_MIN_X100         50u
#define OUTPUT_CURVE_GAMMA_MAX_X100         500u
#define DAC_OUTPUT_MIN_MV                   0u
#define DAC_OUTPUT_MAX_MV                   DAC_FULL_SCALE_MV
#define BLE_DEVICE_NAME                     "Footsie"
#define NVS_NAMESPACE                       "footsie"
#define NVS_KEY_GAMMA                       "gamma_x100"
/* NVS keys for ADC calibration (millivolts) */
#define NVS_KEY_ADC_MIN_MV                  "adc_min_mv"
#define NVS_KEY_ADC_MAX_MV                  "adc_max_mv"
/* NVS keys for output scaling (millivolts) */
#define NVS_KEY_OUTPUT_MIN_MV               "out_min_mv"
#define NVS_KEY_OUTPUT_MAX_MV               "out_max_mv"
/* ADC calibrated range (mV) - adjust these to match your potentiometer's actual measurement range */
/* Set to the calibrated ADC reading at the pot's minimum position */
#define ADC_MIN_CALIBRATED_MV               139
/* Set to the calibrated ADC reading at the pot's maximum position */
#define ADC_MAX_CALIBRATED_MV               3181
#define ADC_SPAN_MV                         (ADC_MAX_CALIBRATED_MV - ADC_MIN_CALIBRATED_MV)

#endif  // SYSTEM_H
