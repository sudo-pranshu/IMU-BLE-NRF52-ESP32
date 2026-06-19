/*
 * XIAO nRF52840 Sense — 6-Axis IMU BLE Sender
 * Streams ax, ay, az, gx, gy, gz at high rate via BLE notify
 * Batch: 20 samples × 12 bytes = 240 bytes per packet (~3.3 kHz accel, 833 Hz gyro)
 */

#include <Arduino.h>
#include <bluefruit.h>
#include <Wire.h>
#include "LSM6DS3.h"

LSM6DS3 imu(I2C_MODE, 0x6A);

// BLE Service / Characteristic UUIDs
BLEService     imuService("ABCD1234-0000-467A-9538-01F0652C74E0");
BLECharacteristic imuChar("ABCD1234-0001-467A-9538-01F0652C74E0", BLENotify, 244);

// ── Batch config ──────────────────────────────────────────────
const uint16_t BATCH_SIZE = 20;          // 20 samples × 12 bytes = 240 bytes
// Layout per sample: ax, ay, az, gx, gy, gz  (each int16_t, little-endian)
int16_t imuBatch[BATCH_SIZE * 6];
uint16_t batchIndex = 0;

// ── Timing ────────────────────────────────────────────────────
// 6.66 kHz ODR on accel; we poll every 150 µs ≈ 6.6 kHz
// Gyro ODR set to 833 Hz; we read it every poll and pack it too
const uint32_t SAMPLE_INTERVAL_US = 150; // ~6.6 kHz
uint32_t lastSampleMicros = 0;

// ── Scale factors ─────────────────────────────────────────────
// Accel: ±4 g  →  1 g = 8192 LSB  (like ±4g int16 convention)
// Gyro:  ±500 dps →  1 dps = 65.536 LSB  (32768 / 500)
const float ACCEL_SCALE = 8192.0f;   // multiply g → int16
const float GYRO_SCALE  = 65.536f;   // multiply dps → int16

void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setName("IMU_6AXIS");
  Bluefruit.setTxPower(4);                    // max TX power

  // Fastest allowed connection interval: 7.5 ms (units of 1.25 ms → 6)
  Bluefruit.Periph.setConnInterval(6, 6);
  Bluefruit.configPrphConn(247, 247, 6, 6);  // MTU 247, data-length 247

  imuService.begin();
  imuChar.begin();

  Bluefruit.Advertising.addService(imuService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.setInterval(32, 32);  // 20 ms fast advertising
  Bluefruit.Advertising.start(0);             // advertise forever
}

void setup() {
  Serial.begin(115200);
  // Don't block on Serial when no USB host — remove while(!Serial) for field use
  delay(500);

  // ── IMU init ──────────────────────────────────────────────
  if (imu.begin() != 0) {
    Serial.println("IMU init FAILED");
    while (1) { delay(100); }
  }

  // Accelerometer
  imu.settings.accelEnabled    = 1;
  imu.settings.accelRange      = 4;    // ±4 g
  imu.settings.accelSampleRate = 6;    // 6 = 6.66 kHz ODR
  imu.settings.accelBandWidth  = 400;  // anti-alias filter 400 Hz

  // Gyroscope
  imu.settings.gyroEnabled     = 1;
  imu.settings.gyroRange       = 500;  // ±500 dps
  imu.settings.gyroSampleRate  = 5;    // 5 = 833 Hz ODR

  // Apply settings by re-calling begin (library requires this)
  imu.begin();

  setupBLE();

  Serial.println("Ready — batching 20 × 6-axis samples per BLE packet");
  lastSampleMicros = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - lastSampleMicros < SAMPLE_INTERVAL_US) return;
  lastSampleMicros = now;

  // Read accel (g) and gyro (dps)
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  // Pack into int16 batch
  uint16_t base = batchIndex * 6;
  imuBatch[base + 0] = (int16_t)(ax * ACCEL_SCALE);
  imuBatch[base + 1] = (int16_t)(ay * ACCEL_SCALE);
  imuBatch[base + 2] = (int16_t)(az * ACCEL_SCALE);
  imuBatch[base + 3] = (int16_t)(gx * GYRO_SCALE);
  imuBatch[base + 4] = (int16_t)(gy * GYRO_SCALE);
  imuBatch[base + 5] = (int16_t)(gz * GYRO_SCALE);

  batchIndex++;

  if (batchIndex >= BATCH_SIZE) {
    // Send 240 bytes: 20 samples × 6 axes × 2 bytes
    if (Bluefruit.connected()) {
      imuChar.notify((uint8_t*)imuBatch, BATCH_SIZE * 12);
    }
    batchIndex = 0;
  }
}
