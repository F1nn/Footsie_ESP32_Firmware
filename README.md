# Footsie

Footsie is a configurable foot pedal controller built around an ESP32-S3, an ADC input from a slide potentiometer, and a 16-bit I2C DAC output.

It reads pedal position, applies a tunable response curve, smooths the result, and drives an analog control voltage for a connected machine or controller.

## Why?

I originally built Footsie for a pottery / ceramics wheel I made for my wife. The commercially available foot pedals I tried were not quite right: low-speed control was poor, the pedal travel was not used effectively and the response did not feel natural.

Footsie exists to solve that problem.

The goal is to provide a foot pedal that can be tuned to behave exactly how you want, especially where fine low-speed control matters.

Yes, this is absolutely overkill.

Just the way i like it.

## Features

* Analog slide potentiometer pedal input on ADC channel 2 / GPIO 3
* Continuous ADC sampling with curve-fitting calibration
* Configurable gamma response curve, persisted in NVS
* Persisted ADC calibration range, also stored in NVS
* Configurable output min/max scaling, persisted in NVS
* 16-bit DAC80501 I2C output stage
* Fail-safe zero-output behavior when input or mapping is invalid
* WS2812 status LED feedback
* BLE GATT service for live monitoring and configuration
* Designed for low-speed precision control where pedal feel matters

## How it works

1. The firmware samples the slide potentiometer continuously through the ADC.
2. Samples are averaged over the update window and converted to millivolts.
3. The calibrated input range is normalized and shaped with a configurable gamma curve.
4. The result is written to the DAC as an analog control voltage.
5. If the ADC, calibration, or DAC path is not valid, the output is driven to zero.
6. A WS2812 LED shows output activity and BLE connection state.

## Purpose

Footsie is not intended to be a universal motor controller. It is a configurable analog control pedal intended to replace or improve simple potentiometer-style foot pedals like you would find on an e-bike or sewing machine, particularly where the stock pedal response is too abrupt, too compressed, or poorly matched to the device being controlled.

**Possible Use Cases:**
- Pottery / ceramics wheels
- Small motor speed control systems
- Sewing machine-style speed pedals
- E-bike-style throttle signal replacement or testing
- Variable analog control inputs
- Custom machine controls
- Any project where a nicer foot-controlled analog voltage is useful

Check the requirements of the device being controlled before connecting Footsie. Not all pedal inputs or throttle inputs are electrically compatible.

## Hardware

### Core Parts

Footsie currently uses these major components:

* ESP32-S3 microcontroller
* DAC80501 16-bit DAC over I2C
* Slide potentiometer as the pedal input
* WS2812B RGB LED for status indication

### Pin Map

The current firmware pin assignments are:

| Function | GPIO |
| --- | --- |
| I2C SDA | 1 |
| I2C SCL | 2 |
| ADC input | 3 |
| RGB LED | 21 |
| UART0 TX | 43 |
| UART0 RX | 44 |

### Current Defaults

| Setting | Value |
| --- | --- |
| ADC reference voltage | 3300 mV |
| DAC full-scale output target | 5000 mV |
| DAC update period | 20 ms |
| Debug log period | 1000 ms |
| Default gamma | 2.2 |
| Gamma range | 0.50 to 5.00 |
| Default calibrated ADC minimum | 139 mV |
| Default calibrated ADC maximum | 3181 mV |

## Schematic

![Footsie Schematic V1.0](docs/schematic/Footsie_Schematic_V1.0.pdf)

## BLE Interface

Footsie exposes a custom BLE GATT service named `Footsie` for live telemetry and configuration.

### Characteristics

| Characteristic | Access | Description |
| --- | --- | --- |
| Gamma | Read / Write | Output curve gamma, stored as an unsigned 16-bit value scaled by 100 |
| ADC calibration | Read / Write | Calibrated minimum and maximum pedal input values in millivolts |
| Output scaling | Read / Write | Output minimum and maximum values in millivolts, scaled over the full pedal travel |
| ADC mV | Read / Notify | Latest averaged pedal input in millivolts |

### BLE Behavior

* The device advertises as `Footsie`.
* Gamma values are accepted in the range 0.50 to 5.00.
* ADC calibration values must satisfy `min < max` and `max <= ADC_VREF_MV`.
* The latest ADC reading is published through GATT notifications when subscribed.
* Advertising is restarted automatically after disconnect or advertising completion.
* If a client disables ADC notifications, the firmware intentionally terminates that BLE connection.

## Configuration

The main tunable values are stored in NVS and survive reboot:

* Output gamma
* Calibrated ADC minimum millivolts
* Calibrated ADC maximum millivolts

The firmware currently uses these defaults if NVS has no saved values:

* Gamma: 2.2
* ADC min: 139 mV
* ADC max: 3181 mV

## Status Indication

The onboard WS2812 LED is used as a simple status display:

* Green intensity tracks the applied DAC output level.
* Blue is lit when BLE is connected and subscribed.
* During startup the LED is briefly set to green when initialization completes.

## ⚡ Building & Flashing

1. **Clone the repository**
```sh
git clone <repository-url>
cd Footsie
```

2. **Set up ESP-IDF**

Follow the normal ESP-IDF setup process for your operating system and editor.

The firmware is built for ESP-IDF with Bluetooth enabled and NimBLE selected.

3. **Build, flash, and monitor**

```sh
./run.sh
```

By default, `run.sh` uses `/dev/ttyACM0`. Override the port if needed:

```sh
PORT=/dev/ttyUSB0 ./run.sh
```

The script runs `idf.py build` and then flashes and opens the serial monitor.

## Development Notes

* ADC sampling is continuous and the firmware averages samples before each DAC update window.
* The DAC output is forced to zero if no valid ADC update is available.
* BLE state is polled from the main loop while the application is running.
* The firmware logs the current mapping from raw input to curved output once per second.

## Safety Disclaimer

Footsie is provided as an experimental DIY project with no warranty or guarantee of safety or fitness for any particular purpose. Use it at your own risk.

Footsie is intended to provide a control voltage only.

It does not provide:

- Electrical isolation
- Motor protection
- Over-current protection
- Emergency stop handling
- Mains safety features
- Mechanical guarding
- Certification for any particular machine or application

You are responsible for ensuring any system using Footsie is safe, suitable, and compliant with applicable laws and regulations.

## License

MIT License (see LICENSE file)
