# ioT-Based Single Phase Smart Energy Meter Using Dual-Core Edge Processing

A Dual-Core FreeRTOS-based single-phase smart energy meter built on the ESP32.

## Overview
This project continuously monitors power consumption (Voltage, Current, Real Power, Power Factor, and Accumulated Energy) using high-frequency DMA ADC sampling and 64-bit DSP math on Core 1 of the ESP32. Core 0 handles Wi-Fi connectivity and telemetry data synchronization to the Blynk IoT Cloud, providing real-time data visualization and persistent energy accumulation.

## Key Features
- **Dual-Core Architecture:** Segregates heavy math calculations from IoT networking to ensure reliable and uninterrupted sampling.
- **High-Frequency DMA Sampling:** Reads raw voltage and current data at 20kHz via hardware DMA.
- **DSP Math & Phase Correction:** Real-time calculation of RMS Voltage, RMS Current, Real Power, and Power Factor with hybrid phase correction for improved accuracy.
- **Blynk IoT Integration:** Live data streaming to the Blynk app/dashboard, including cloud-persisted energy accumulation.
- **Push Notifications:** Automatic alerts triggered for high power usage and energy thresholds.

---

## Wiring Guide

### Hardware Required
- ESP32 Development Board
- ZMPT101B AC Voltage Sensor Module
- ACS712 AC Current Sensor Module
- AC Load and Power Source

### Pin Mapping
The project uses the ESP32's ADC1 channels for continuous DMA sampling.

| Component | ESP32 Pin | ADC Channel | Function |
| :--- | :--- | :--- | :--- |
| ZMPT101B (Voltage) | **GPIO 32** | ADC1_CHANNEL_4 | Measures AC Voltage |
| ACS712 (Current) | **GPIO 33** | ADC1_CHANNEL_5 | Measures AC Current |

### Important Safety Warning
**DANGER:** This project involves working with mains AC voltage (110V/220V), which can be lethal. 
- Ensure all AC wiring is securely insulated and enclosed.
- Never touch the circuit while it is plugged into the wall.
- Ensure the ZMPT101B and ACS712 modules are correctly isolated.

---

## Implementation Details

The firmware utilizes ESP-IDF/FreeRTOS running within the Arduino core. The workload is strictly partitioned between the two ESP32 cores to achieve accurate, uninterrupted power measurements.

### Core 1 (APP_CPU): DSP & Math Task
- **DMA ADC:** The `adc_continuous` API is configured to read from GPIO 32 and GPIO 33 at a sample rate of 20kHz (10k per channel).
- **DSP Math:** As the DMA buffer fills, Core 1 processes the data chunks (200ms windows). It squares the samples and multiplies voltage by current to calculate the `vRms`, `iRms`, `realP` (Real Power), and `pf` (Power Factor).
- **Phase Correction:** A hybrid phase correction mechanism consisting of a macro-shift (array indexing offset) and micro-shift (fractional interpolation) aligns the voltage and current waveforms to mitigate hardware-induced phase delays.
- **IPC (Inter-Process Communication):** Once a 200ms calculation is complete, the results are packaged into a struct and pushed to a FreeRTOS Queue.

### Core 0 (PRO_CPU): IoT & Network Task
- **Blynk Cloud:** Connects to the local Wi-Fi and authenticates with the Blynk Cloud.
- **Data Consumption:** Reads the calculated data from the FreeRTOS Queue.
- **Energy Accumulation:** Accumulates Total Energy (Wh) and syncs the value with the Blynk server (Virtual Pin V5) to ensure it survives reboots.
- **Telemetry:** Pushes telemetry data (V, I, W, PF) to the Blynk dashboard every 2 seconds.
- **Alerts:** Evaluates power thresholds (e.g., >80W limit) and triggers push notifications through Blynk.

---

## Setup & Configuration

### 1. Prerequisites
- ESP32 Development Board.
- Arduino IDE or PlatformIO installed.
- ESP32 board package installed in your IDE.
- [Blynk IoT App](https://blynk.io/) (iOS/Android) and an active account.

### 2. Library Dependencies
Make sure the following libraries are installed:
- `WiFi.h` (Built-in)
- `BlynkSimpleEsp32.h` (Available via Library Manager)

### 3. Blynk Configuration
1. Create a new Template in the Blynk Web Console:
   - Name: `Single Phase Smart Meter` (or your preferred name)
   - Hardware: `ESP32`
   - Connection Type: `WiFi`
2. Create the following Datastreams (Virtual Pins):
   - **V1:** Voltage (Double)
   - **V2:** Current (Double)
   - **V3:** Power (Double)
   - **V4:** Power Factor (Double)
   - **V5:** Energy (Double)
3. Copy your Template ID, Template Name, and Auth Token.
4. Replace the credentials at the top of `SinglePhase.ino`:
   ```cpp
   #define BLYNK_TEMPLATE_ID "Your_Template_ID"
   #define BLYNK_TEMPLATE_NAME "Your_Template_Name"
   #define BLYNK_AUTH_TOKEN "Your_Auth_Token"
   ```

### 4. Wi-Fi Configuration
Update your local Wi-Fi credentials in the code:
```cpp
char ssid[] = "Your_WiFi_SSID";
char pass[] = "Your_WiFi_Password";
```

### 5. Calibration
The constants `VCAL`, `ICAL`, `SAMPLE_SHIFT`, and `PHASECAL` at the top of the file determine measurement accuracy. They are currently tuned for a 100W bulb at ~235V.
To recalibrate:
1. Use a known resistive load (e.g., an incandescent bulb) and a multimeter.
2. Adjust `VCAL` so the serial output matches your multimeter's AC voltage reading.
3. Adjust `ICAL` so the serial output matches your multimeter's AC current reading.
4. Adjust `PHASECAL` and `SAMPLE_SHIFT` to ensure the Power Factor reads exactly `1.0` for a pure resistive load.

### 6. Flashing
Upload the code to your ESP32. Open the Serial Monitor (115200 baud) to view the DC Offset auto-calibration and real-time power data.
