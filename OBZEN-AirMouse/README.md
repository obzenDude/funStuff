# OBZEN-AirMouse

Wireless Bluetooth Low Energy (BLE) HID air mouse firmware for the **Seeed XIAO nRF52840 Sense** (`XIAO-nRF52840-AirMouse`), powered by its onboard **LSM6DS3** 6-axis IMU (`0x6A`) and **Adafruit Bluefruit nRF52** (`BLEHidAdafruit`).

## Overview

`OBZEN-AirMouse` turns a Seeed XIAO nRF52840 Sense into a motion-controlled wireless mouse over Bluetooth Low Energy. Gyroscope and accelerometer readings from the onboard LSM6DS3 are filtered, leveled for roll tilt, mapped through a deadzone and sensitivity/acceleration curve, and sent as standard BLE HID mouse reports (`dx`, `dy`)—requiring no companion receiver or Python script on the host computer for cursor movement.

## Features

- **Direct BLE HID Mouse:** Advertises as `OBZEN-AirMouse` (`BLE_APPEARANCE_HID_MOUSE`) and pairs directly with your laptop's built-in Bluetooth stack.
- **Grip & Orientation Detection:** Automatically switches between horizontal (`HORIZ`) and upright (`UPRIGHT`) grips (`AUTO_ORIENTATION`) with debounce (`ORIENT_HOLD_MS`) or locks to a chosen grip (`MANUAL_HOLD_MODE`).
- **Tilt-Angle Compensation:** Leveling logic in horizontal grip uses roll angle (`atan2(ay, az)` scaled by `TILT_COMP_GAIN`) to reduce cross-axis coupling when tilted.
- **Sensitivity & Drift Control:** Uses gyroscope bias calibration (`calibrateGyro()`), deadzone filtering (`DEADZONE`), low-pass smoothing (`SMOOTHING`), base sensitivity (`BASE_SENSITIVITY`), acceleration gain (`ACCEL_GAIN`), and vertical scaling (`PITCH_SCALE`).
- **Auto-Sleep & Wake-on-Motion:** Enters low-power sleep (`sd_power_system_off()`) after 30 seconds of stillness (`IDLE_SLEEP_MS`) and configures the IMU wake-up interrupt (`WAKE_PIN`) to wake on motion.
- **Serial Telemetry:** Prints periodic status lines (`grip:`, `maxDeltaG:`, `maxGyro:`, `idleSec:`) at `115200` baud.

## Hardware & Libraries

- **Board:** Seeed XIAO nRF52840 Sense (with onboard LSM6DS3 IMU)
- **Headers / Libraries Used in `AirMouse.ino`:**
  - `bluefruit.h` (`BLEHidAdafruit`, `BLEDis`) — nRF52 BLE stack and HID mouse profile
  - `LSM6DS3.h` — onboard 6-axis IMU driver
  - `Wire.h` — I2C communication
  - `math.h` — standard math functions (`atan2`, `cos`, `sin`, `fabs`)
  - `nrf_gpio.h` — nRF52 GPIO / wake-up pin hardware support

> Note: These headers come from your Arduino board package and installed libraries—they do not need to be copied into this repository.

## Folder Structure

```text
OBZEN-AirMouse/
├── AirMouse.ino
└── README.md
