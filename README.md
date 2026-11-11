# Low-Power Environmental Monitor (RP2040 + SHT40)

An autonomous environmental monitoring system built on the Raspberry Pi Pico (RP2040) using a Sensirion SHT40 temperature and humidity sensor via hardware I2C.

## Features
- **Duty Cycle:** 10 seconds Active sampling (1 Hz) / 60 seconds Low-Power Sleep.
- **Adaptive Clock Scaling:** Drops the core clock from 125 MHz to 12 MHz (Crystal) during sleep, cutting current from 23.4 mA to 2.54 mA.
- **Automatic Bus Recovery:** Features a 9-clock recovery mechanism to unstick I2C lines if a sensor communication glitch occurs.
- **Data Validation:** Uses CRC-8 checksum verification on all temperature and humidity readings.

## Power Measurements
- **Active Window (10s):** ~23.4 mA (125 MHz, LED ON)
- **Sleep Window (60s):** ~2.54 mA (12 MHz Crystal, PLLs disabled)

## Wiring Connections
- Pico GP4 (Pin 6) -> SHT40 SDA (Yellow wire)
- Pico GP5 (Pin 7) -> SHT40 SCL (Green wire)
- Pico 3V3 (Pin 36) -> SHT40 VCC (Red wire)
- Pico GND (Pin 38) -> SHT40 GND (Black wire)