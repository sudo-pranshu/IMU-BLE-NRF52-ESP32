# BLE IMU Data Streaming (nRF52840 → ESP32 → Serial)

This project streams real-time IMU (accelerometer) data from a Seeed Studio XIAO nRF52840 Sense to an ESP32 over BLE using notifications. The ESP32 receives, timestamps, and forwards the data to a PC via serial for logging and analysis.

## 📦 Components Used

- ✅ Seeed Studio XIAO nRF52840 Sense  
- ✅ ESP32 Dev Board (e.g., NodeMCU ESP32-S v1.1)  
- ✅ LSM6DS3 (built-in on XIAO nRF52840 Sense)  
- ✅ USB connection to PC (for serial logging)

## ⚙️ How It Works

1. **nRF52840 (Peripheral)**  
   - Reads IMU accelerometer data from LSM6DS3.
   - Packs multiple samples into a single BLE notification.
   - Sends batched data via BLE notifications to ESP32.
   - Uses `micros()` on ESP32 only for high-accuracy timestamps.

2. **ESP32 (Central)**  
   - Connects to the nRF52840 via BLE.
   - Subscribes to notifications.
   - Timestamps data upon receipt using `micros()` and sends it to PC over Serial.


## 📁 File Structure

nrf-175-sps.ino       → Code for Seeed XIAO nRF52840 Sense (Peripheral)  
esp-175-sps.ino       → Code for ESP32 (Central)  
README.md             → Project documentation

## Key Features

- BLE Notifications for high throughput  
- Batching: Sends 20 IMU samples per BLE packet  
- High-speed sampling: ~175 samples/sec  
- Accelerometer only (ax, ay, az)  
- Timestamping on ESP32 using `micros()`  

## Requirements

**Arduino Libraries**  
Install via Arduino Library Manager:
- SparkFun LSM6DS3
- NimBLE-Arduino (by h2zero)

**Board Support**  
- Seeed nRF52 Boards package  
- ESP32 Boards by Espressif

## Usage

1. Upload `nrf-175-sps.ino` to Seeed XIAO nRF52840 Sense.  
2. Upload `esp-175-sps.ino` to ESP32.  
3. Open Serial Monitor (115200 baud) on ESP32.  
4. Capture and log incoming data to CSV using any tool (Python, CoolTerm, etc.).

## Sample CSV Format

micros,ax,ay,az  
103204512,128,-456,980  
103204617,130,-460,978  
...

## Author

Pranshu Kumar  
IoT Research Intern, Machine Intelligence Program  
GitHub: https://github.com/sudo-pranshu


