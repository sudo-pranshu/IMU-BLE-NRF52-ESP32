#include <Arduino.h>
#include <bluefruit.h>
#include <Wire.h>
#include "LSM6DS3.h"

LSM6DS3 imu(I2C_MODE, 0x6A); // I2C configuration

// BLE Configuration
BLEService imuService("ABCD1234-0000-467A-9538-01F0652C74E0");
BLECharacteristic imuChar("ABCD1234-0001-467A-9538-01F0652C74E0", BLENotify, 244);

// Batch parameters
const uint16_t BATCH_SIZE = 20; // 20 samples * 12 bytes = 240 bytes
float imuBatch[BATCH_SIZE * 3]; // x,y,z for 20 samples
uint16_t batchIndex = 0;
uint32_t lastSampleMicros = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    
    // Initialize IMU at 6.66kHz
    if(imu.begin() != 0) {
        Serial.println("IMU init failed!");
        while(1);
    }
    imu.settings.accelEnabled = 1;
    imu.settings.accelRange = 16;
    imu.settings.accelSampleRate = 6; // 6 = 6.66kHz mode
    imu.settings.accelBandWidth = 400;

    // BLE Setup
    Bluefruit.begin();
    Bluefruit.setName("IMU_6K6");
    Bluefruit.Periph.setConnInterval(6, 6); // 7.5ms connection interval
    Bluefruit.configPrphConn(247, 247, 6, 6); // Max MTU
    
    imuService.begin();
    imuChar.begin();
    
    Bluefruit.Advertising.addService(imuService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.start(0);
    
    Serial.println("Ready! Sampling at 6.66kHz");
    lastSampleMicros = micros();
}

void loop() {
    // Maintain 6.66kHz sampling using micros()
    if(micros() - lastSampleMicros >= 150) { // 6666Hz = ~150µs interval
        lastSampleMicros = micros();
        
        // Read and store sample
        imuBatch[batchIndex * 3 + 0] = imu.readFloatAccelX();
        imuBatch[batchIndex * 3 + 1] = imu.readFloatAccelY();
        imuBatch[batchIndex * 3 + 2] = imu.readFloatAccelZ();
        batchIndex++;

        // Send batch when full
        if(batchIndex >= BATCH_SIZE) {
            imuChar.notify((uint8_t*)imuBatch, BATCH_SIZE * 12);
            batchIndex = 0;
        }
    }
}
