
# funStuff

A collection of experimental embedded firmware, sensor, and visualization projects.

## Projects

- **[OBZEN-AirMouse](./OBZEN-AirMouse)** — Bluetooth Low Energy (BLE) HID air mouse firmware (`AirMouse.ino`) for the Seeed XIAO nRF52840 Sense using its onboard LSM6DS3 IMU (`BLEHidAdafruit`), with tilt compensation, grip detection, sensitivity/deadzone tuning, and 30-second sleep / wake-on-motion.
- **[OBZEN-GyroSense](./OBZEN-GyroSense)** — IMU BLE telemetry and 3D visualization project.

## General Notes

- Each project folder contains its own source files and project-specific `README.md` with exact setup, library, and tuning details.
- Make sure only one host connection or serial/BLE client accesses the board at a time during testing.

## Repository Structure

```text
funStuff/
├── OBZEN-AirMouse/
│   ├── AirMouse.ino
│   └── README.md
└── OBZEN-GyroSense/
