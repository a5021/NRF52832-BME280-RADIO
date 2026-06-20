# NRF52832-BME280-RADIO

[![Build](https://github.com/a5021/NRF52832-BME280-RADIO/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/NRF52832-BME280-RADIO/actions/workflows/build.yml)

[![MCU](https://img.shields.io/badge/MCU-nRF52832-00A9E0)](https://www.nordicsemi.com/Products/nRF52832)
[![Sensor](https://img.shields.io/badge/Sensor-BME280-00A9E0)](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
[![Radio](https://img.shields.io/badge/Radio-2.4GHz_2Mbit-00A9E0)]()
[![IDE](https://img.shields.io/badge/IDE-Make%20%7C%20EWARM%20%7C%20MDK--ARM%20%7C%20SES-00A9E0)]()
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

Bare-metal firmware for an nRF52832-based wireless environmental sensor node. Reads Bosch BME280 (temperature, pressure, humidity) and on-chip die temperature, then transmits the data packet over a 2.4 GHz proprietary radio link.

## Features

- Register-level, non-blocking peripheral drivers (TWI, RADIO, SAADC, RTC, TEMP)
- Fully compensated BME280 readings using int64 arithmetic
- SAADC VDD monitoring — 12-bit, 64x oversampling, offset calibration
- nRF52 on-chip die temperature measurement
- 60-second periodic transmission interval (RTC2-driven)
- System ON idle with `__WFE()` sleep between cycles; HFCLK gated off
- Optional UART debug output (`-DUSE_UART`, 9600 baud, P0.06)
- Errata workarounds applied: PAN_028 #29, #30, #31

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | Nordic nRF52832 (ARM Cortex-M4F @ 64 MHz) |
| Sensor | Bosch BME280 (I²C address 0x76) |
| Radio | 2 Mbps GFSK, channel 99 (2479 MHz), +4 dBm TX |
| TX Period | 60 seconds (RTC2 CC[0] = 983 040 ticks) |
| Power | 2× AA / Li-Po, measured via SAADC VDD input |

## Pin Assignment

| Signal | Pin  | Notes                     |
|--------|------|---------------------------|
| SDA    | P0.26| TWI1, 400 kHz             |
| SCL    | P0.27| TWI1                      |
| UART TX| P0.06| Optional, 9600 8N1        |

## Radio Protocol — Packet Format

10-byte payload transmitted as a single RADIO packet (2Mbit mode, 16-bit CRC):

| Offset | Size | Field     | Description                              |
|--------|------|-----------|------------------------------------------|
| 0      | 1    | Length    | Payload length, fixed to 10              |
| 1      | 1    | Seq ID    | Rolling counter, bit 0 always set        |
| 2      | 3    | Pressure  | Compensated pressure, Pa (24-bit)        |
| 5      | 1    | Die Temp  | nRF52 junction temperature, °C ÷ 4       |
| 6      | 2    | Temp      | BME280 temperature, 0.01 °C             |
| 8      | 2    | Humidity  | BME280 relative humidity, 0.1 %RH       |
| 10     | 2    | Voltage   | VDD battery, 3600 / 4095 mV per LSB     |

Address field: base `0xE7E7E7E7`, prefix `0xE7` (full address `0xE7E7E7E7E7`).

## Firmware Architecture

```
+----------+    +----------+    +-----------+    +-----------+    +-----------+
|  CLOCK   |--->|   TWI1   |--->|  BME280   |--->|Compensate |--->|  RADIO    |
| HFXO/LFXO|    | SDA/SCL  |    | 0x76 I2C  |    | int64_t   |    | ch.99 2M  |
+----------+    +----------+    +-----------+    +-----------+    +-----------+
                                                     |
                                               +-----v------+
                                               |   SAADC    |
                                               |  VDD meas  |
                                               +------------+
                                                     |
                                               +-----v------+
                                               |   TEMP     |
                                               | die temp   |
                                               +------------+
                                                     |
                                               +-----v------+
                                               |   RTC2     |
                                               | 60 s wake  |
                                               +------------+
```

## Getting Started

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`) >= 9-2020-q2-update
- GNU Make
- (Optional) nRF5x command-line tools for flashing: `nrfjprog`

### Build & Flash

```console
$ make
$ make jflash                # nRF52-DK / J-Link
$ make stflash               # ST-LINK (ST-LINK_CLI.exe)
```

### UART Debug

Define `USE_UART` at build time to enable formatted output on P0.06 (9600 baud):

```console
$ make CFLAGS="-DUSE_UART"
```

## Supported IDEs

| IDE                     | Project path             |
|-------------------------|--------------------------|
| SEGGER Embedded Studio  | `IDE/ses/`               |
| IAR EWARM               | `IDE/EWARM/`             |
| Keil MDK-ARM (uVision)  | `IDE/MDK-ARM/`           |

## Project Structure

```
+-- main.c                 Application entry point and main loop
+-- Makefile               GCC build system
+-- .travis.yml            CI configuration
+-- LICENSE                MIT License
+-- drv/
|   +-- inc/               CMSIS-CORE + nRF52 register definitions
|   |   +-- CMSIS/         core_cm4.h, cmsis_gcc.h, cmsis_iccarm.h
|   |   +-- nrf.h          Master MCU header selector
|   |   +-- nrf52.h        nRF52832 peripheral struct definitions
|   |   +-- nrf52_bitfields.h  Bit-field enumerations (SVD-generated)
|   +-- src/
|       +-- gcc_startup_nrf52.S  Reset vector, data/BSS init
|       +-- system_nrf52.c       SystemInit, errata workarounds
|       +-- iar_startup_nrf52.s  IAR startup
+-- IDE/
    +-- ses/               SEGGER Embedded Studio project
    +-- EWARM/             IAR Embedded Workbench project
    +-- GCC/               Linker script + Makefile templates
    +-- MDK-ARM/           Keil uVision project
```

## Known Errata Workarounds

| Anomaly | Description              | Workaround                        |
|---------|--------------------------|-----------------------------------|
| PAN_028 #29 | TEMP STOP clears register | Read TEMP before TASKS_STOP       |
| PAN_028 #30 | TEMP AFE not powered down | Explicit TASKS_STOP after DATARDY |
| PAN_028 #31 | TEMP offset uninitialized | Manual load to `0x4000C504`       |

## License

Distributed under the MIT License. See [LICENSE](LICENSE) for details.