# ESP32 BLE Driving Controller for macOS

A DIY driving controller (steering wheel + three pedals) built on an **ESP32** that connects over **Bluetooth Low Energy** as a standard HID gamepad. It was built to solve one problem: **racing wheels are not properly supported on macOS**, so playing truck and driving simulators such as Euro Truck Simulator 2 on a Mac with a wheel is difficult or impossible with commercial hardware.

## The Problem

Most commercial steering wheels and pedal sets ship with drivers and configuration tools for Windows only. On macOS they are often not detected at all, detected with missing axes, or detected without force feedback or calibration options. There is usually no way to fix this on the software side.

## The Solution

Instead of relying on a vendor driver, this project makes the ESP32 present itself as a generic BLE gamepad. macOS and its Gamepad API already support standard HID gamepads, so the wheel and pedals show up as plain axes that any game can bind to, with no driver required.

A potentiometer is used for steering, and three more analog inputs read the accelerator, brake, and clutch pedals.

## Axis Mapping

| HID Axis | Function | Output Range | GPIO |
|---|---|---|---|
| X | Brake | 0 - 32767 | 32 |
| Y | Accelerator | 0 - 32767 | 34 |
| Z | Steering | 0 - 32767 | 33 |
| RX | Clutch | 0 - 32767 | 35 |

Steering is mapped with its center at the middle of the axis:

| Wheel Position | Axis Value |
|---|---|
| Full left | 0 |
| Center (straight) | 16383 |
| Full right | 32767 |

The center point is read automatically when the ESP32 powers on, so **turn the board on with the wheel pointing straight ahead**.

### Why steering uses 0 - 32767 and not a signed range

The first version sent a bipolar range (-32767 to +32767) on the steering axis. In the game this produced wrong readings: left and center were swapped or jumped unpredictably.

The cause is that BLE gamepad axes are declared with a logical minimum of 0, so the host reads each 16-bit value as **unsigned**. A negative number such as -1 is then interpreted as 65535, which is outside the declared range and gets clamped or wrapped by macOS. Keeping every axis inside 0 - 32767 and treating 16383 as the center fixes this.

## Hardware

- ESP32 development board (all inputs use ADC1 pins, so they keep working while BLE is active)
- 3 potentiometers or analog pedal sensors (accelerator, brake, clutch)
- 1 potentiometer for steering

Wire the outer pins of each potentiometer to **3.3V** and **GND**, and the middle pin to the GPIO listed in the table above.
<img width="993" height="605" alt="Screenshot 2026-10-11 at 22 25 18" src="https://github.com/user-attachments/assets/c3b07af1-39f0-4262-ac76-ad807af523f4" />
<img width="960" height="545" alt="Screenshot 2026-10-11 at 22 25 32" src="https://github.com/user-attachments/assets/147f2f22-4e6c-407c-bffb-059eed8ace2d" />

## Software

- Arduino IDE or PlatformIO with ESP32 board support
- [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) library (with NimBLE-Arduino)

## Setup

1. Install the required library.
2. Open `main.cpp`, adjust pins and calibration values if needed, and upload it to the ESP32.
3. Power the board with the steering wheel straight.
4. On macOS, open **System Settings > Bluetooth** and connect to **v8 Driving**.
5. In your game, assign each control by moving the wheel or pressing a pedal.

If you change the firmware or the axis layout, **forget the device in macOS Bluetooth settings and pair again**. macOS caches the HID descriptor, and the old one can cause wrong or missing axes.

## Calibration

Edit these constants at the top of the code to match your potentiometers. Open the Serial Monitor at **115200 baud** to read the raw values.

```cpp
const int ACCELERATOR_MIN = 1700;
const int ACCELERATOR_MAX = 2350;

const int BRAKE_MIN = 2000;
const int BRAKE_MAX = 2700;

const int CLUTCH_MIN = 1500;
const int CLUTCH_MAX = 2120;

const int STEERING_MIN = 250;
const int STEERING_MAX = 3600;
```

Set each `MIN` with the pedal released (or the wheel fully left) and each `MAX` with the pedal fully pressed (or the wheel fully right). If an axis moves in the wrong direction, set the matching `INVERT_*` constant to `true`.

## Signal Filtering

| Parameter | Default | Purpose |
|---|---|---|
| `ADC_SAMPLES` | 16 | Samples averaged per reading |
| `DEAD_ZONE` | 25 | Ignores small pedal changes (noise) |
| `SMOOTHING_ALPHA` | 0.3 | Pedal smoothing (lower is smoother) |
| `STEERING_DEAD_ZONE` | 3 | Small dead zone to keep steering fluid |
| `STEERING_SMOOTHING_ALPHA` | 0.65 | Steering smoothing (more responsive) |

HID reports are only sent when a value actually changes.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Steering jumps or reads incorrectly in game | Make sure no axis value is negative, then forget the device and pair again |
| Steering is not straight at neutral | Restart the board with the wheel pointing straight |
| Axis moves in the wrong direction | Toggle the matching `INVERT_*` constant |
| Pedal values jitter | Increase `DEAD_ZONE` or `ADC_SAMPLES` |
| Device does not appear in Bluetooth | Check the Serial Monitor for `BLE Gamepad started` |

## License

**Personal use only. Commercial use is not permitted.**

You may build, modify, and use this project for your own personal, non-commercial purposes. You may not sell this project, its firmware, or devices built from it, and you may not use it in any commercial product or service.

This project depends on third-party libraries (such as ESP32-BLE-Gamepad), which are distributed under their own licenses. Those licenses continue to apply to the respective libraries.

Copyright (c) 2026. All rights reserved except for the personal use granted above.
