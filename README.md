# SCD41 Sensor Display System

This repository contains embedded C firmware for interfacing a **Sensirion SCD41 CO₂ / temperature / humidity sensor** with a microcontroller and displaying the measured values on a **character/serial LCD**. The project uses **HAL-based abstractions** to keep the sensor and display drivers portable across hardware platforms.

---

## Project Overview

The firmware performs the following tasks:

- Communicates with the **SCD41 sensor over I²C**
- Reads **CO₂ concentration, temperature, and humidity**
- Formats sensor data into human-readable strings
- Displays the data on an **LCD display**
- Uses **Hardware Abstraction Layers (HALs)** to separate hardware-specific code from application logic

---

## Repository Structure

```
.
├── main.c
├── scd41.c / scd41.h
├── scd4x_i2c.c / scd4x_i2c.h
├── sensirion_common.c / sensirion_common.h
├── sensirion_config.h
├── sensirion_i2c.c / sensirion_i2c.h
├── sensirion_i2c_hal.c / sensirion_i2c_hal.h
├── sensiron_HAL.c
├── serlcd.c / serlcd.h
└── README.md
```

---

## How It Works

1. **Initialization**
   - HAL drivers initialize I²C and required peripherals
   - LCD display is initialized and cleared

2. **Sensor Setup**
   - The SCD41 sensor is initialized using Sensirion’s driver API
   - Measurement mode is configured

3. **Measurement Loop**
   - CO₂, temperature, and humidity values are read over I²C
   - Values are formatted into strings
   - Data is sent to the LCD using the `serlcd` driver

4. **Periodic Update**
   - The display refreshes at a fixed interval

---

## Example LCD Output

```
CO2:  421 ppm
Temp: 24.1 C
Humi: 46.3 %
```
[![SCD41 Sensor Display Demo](demo.mp4)

---

## HAL Design

- Sensor logic is platform-agnostic
- I²C operations are implemented in `sensirion_i2c_hal.c`
- Display control is abstracted through `serlcd.c`

This separation allows easy porting to different microcontrollers.

---


## Hardware Requirements

- Microcontroller with **I²C support**
- **Sensirion SCD41** sensor
- **Character or serial LCD**
- Proper I²C pull-up resistors

---

## Notes

- Verify I²C wiring and voltage levels
- Adjust HAL implementations for your target platform
- Display formatting can be modified in `main.c`
