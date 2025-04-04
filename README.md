# README

## Overview

This project is an ESP32-based power management system designed to control relays based on ignition signals and battery voltage. It includes functionality for reading and processing commands via serial communication, saving and loading settings to non-volatile memory, and sending status updates periodically.

## Features

- **Relay Control**: Controls two relays (`PIN_RELAY1` and `PIN_RELAY2`) based on ignition signal (`PIN_IGNITION`) and battery voltage (`PIN_BATTERY`).
- **Battery Voltage Monitoring**: Reads and averages battery voltage using an ADC pin and a calibration factor.
- **Settings Persistence**: Saves and loads configurable settings (e.g., minimum voltage, relay timings, and device ID) using the ESP32's `Preferences` library.
- **Serial Communication**: Processes commands received via serial input and sends status updates.
- **Configurable Parameters**:
  - Minimum voltage threshold (`minVoltage`)
  - Relay 1 and Relay 2 activation times (`relay1Time`, `relay2Time`)
  - Device ID (`deviceId`)

## Hardware Requirements

- ESP32 development board
- Relays connected to GPIO pins 22 and 23
- Ignition signal connected to GPIO pin 35
- Battery voltage connected to ADC pin 34

## Software Requirements

- Arduino framework
- PlatformIO for building and uploading the firmware

## Pin Configuration

| Pin Name      | GPIO | Description                  |
|---------------|------|------------------------------|
| `PIN_IGNITION`| 35   | Input for ignition signal    |
| `PIN_RELAY1`  | 22   | Output for relay 1           |
| `PIN_RELAY2`  | 23   | Output for relay 2           |
| `PIN_BATTERY` | 34   | ADC input for battery voltage|

## How It Works

1. **Setup**:
   - Initializes serial communication at 115200 baud.
   - Configures GPIO pins for input/output.
   - Loads saved settings from non-volatile memory.
   - Initializes the voltage buffer with the initial battery voltage reading.

2. **Main Loop**:
   - Reads the ignition signal and battery voltage.
   - Updates the voltage buffer and calculates the average battery voltage.
   - Controls relays based on ignition state and battery voltage.
   - Sends periodic status updates via serial communication.
   - Processes incoming serial commands to update settings.

3. **Serial Commands**:
   - Commands are received in the format `KEY:VALUE`.
   - Supported commands:
     - `MIN_VOLTAGE:<value>`: Sets the minimum voltage threshold.
     - `RELAY1_TIME:<value>`: Sets the activation time for relay 1 (in minutes).
     - `RELAY2_TIME:<value>`: Sets the activation time for relay 2 (in minutes).
     - `DEVICE_ID:<value>`: Sets the device ID.

4. **Status Updates**:
   - Sends the current state of the system, including ignition status, battery voltage, relay states, and configurable parameters.

## Example Serial Commands

- Set minimum voltage to 11.5V:
  ```
  MIN_VOLTAGE:11.5
  ```
- Set relay 1 activation time to 2 minutes:
  ```
  RELAY1_TIME:2
  ```
- Set device ID to `DEVICE_5678`:
  ```
  DEVICE_ID:DEVICE_5678
  ```

## Configurable Parameters

| Parameter       | Default Value | Description                          |
|------------------|---------------|--------------------------------------|
| `minVoltage`     | 10.0          | Minimum battery voltage threshold    |
| `relay1Time`     | 1 minute      | Relay 1 activation time (in minutes)|
| `relay2Time`     | 1 minute      | Relay 2 activation time (in minutes)|
| `deviceId`       | `DEVICE_1234` | Unique identifier for the device     |

## Dependencies

- **Arduino Framework**: Provides core functionality for the ESP32.
- **Preferences Library**: Used for saving and loading settings to non-volatile memory.

## Building and Uploading

1. Install [PlatformIO](https://platformio.org/).
2. Open the project in Visual Studio Code.
3. Build and upload the firmware using the following command:
   ```sh
   pio run --target upload
   ```