# Footsie

---

Footsie is a 16-bit DAC output foot pedal using an analog slide potentiometer as the input source.

1. Firmware reads the slide potentiometer via ADC,
2. Computes input->output transform from customisable curve,
3. Outputs a smooth, filtered output via 16-bit DAC over I2C.
4. DAC will return-to-zero output voltage as fail-safe.

---

## 🏠 Features


---

## 🔌 Schematic

![Footsie Schematic V1.0](schematic/Footsie_Schematic_V1.0.pdf)

---

## 🔩 Hardware Components

### ESP32-S3 Microprocessor

* Dual-core Xtensa LX7 @ up to 240 MHz.
* 512 KB SRAM + 384 KB ROM (on-chip) + 8MB PSRAM on module.
* Integrated Wi-Fi 4 (802.11 b/g/n) + Bluetooth 5 (LE).
* USB-C OTG support for power, firmware updates, etc.
* Deep sleep as low as ~5 µA.

---

## ✅ TODO

- [ ] Add web/app interface for configuration
- [ ] Introduce a proper state machine to handle the different modes of operation
- [ ] Documentation

---

## ⚡ Building & Flashing

1. **Clone the repository.**
2. **Configure ESP-IDF and your favourite code editor like VSCode.**
3. **Build / flash and monitor in one step:**

```sh
./run.sh
```

---

## 📄 License

MIT License (see LICENSE file)

---
