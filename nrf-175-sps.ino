#include <Arduino.h>
#include <bluefruit.h>
#include <Wire.h>
#include "LSM6DS3.h"

LSM6DS3 imu(I2C_MODE, 0x6A); // I2C

BLEService imuService("ABCD1234-0000-467A-9538-01F0652C74E0");
BLECharacteristic imuChar("ABCD1234-0001-467A-9538-01F0652C74E0", BLENotify, 244);

// Batch settings
const uint16_t BATCH_SIZE = 40;
int16_t imuBatch[BATCH_SIZE * 3]; // x, y, z each 2 bytes
uint16_t batchIndex = 0;
uint32_t lastSampleMicros = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (imu.begin() != 0) {
    Serial.println("IMU init failed!");
    while (1);
  }

  // Max config for accel
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 4; // Use ±4g for more precision
  imu.settings.accelSampleRate = 6; // 6 = 6.66kHz
  imu.settings.accelBandWidth = 400;

  Bluefruit.begin();
  Bluefruit.setName("IMU_6K6");
  Bluefruit.Periph.setConnInterval(6, 6); // 7.5 ms
  Bluefruit.configPrphConn(247, 247, 6, 6);

  imuService.begin();
  imuChar.begin();

  Bluefruit.Advertising.addService(imuService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start(0);

  Serial.println("Ready! Batching 40 samples.");
  lastSampleMicros = micros();
}

void loop() {
  if (micros() - lastSampleMicros >= 150) {
    lastSampleMicros = micros();

    float x = imu.readFloatAccelX();
    float y = imu.readFloatAccelY();
    float z = imu.readFloatAccelZ();

    // Scale from g to int16 (example: ±4g → 32767)
    imuBatch[batchIndex * 3 + 0] = (int16_t)(x * 8192); 
    imuBatch[batchIndex * 3 + 1] = (int16_t)(y * 8192);
    imuBatch[batchIndex * 3 + 2] = (int16_t)(z * 8192);
    batchIndex++;

    if (batchIndex >= BATCH_SIZE) {
      imuChar.notify((uint8_t*)imuBatch, BATCH_SIZE * 6);
      batchIndex = 0;
    }
  }
}
