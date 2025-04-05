#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

BLEUUID serviceUUID("ABCD1234-0000-467A-9538-01F0652C74E0");
BLEUUID charUUID("ABCD1234-0001-467A-9538-01F0652C74E0");

uint32_t totalSamples = 0;
uint32_t lastReportMicros = 0;
uint32_t baseMicros = 0;

String formatTimestamp(uint32_t microsOffset) {
    uint32_t totalMicros = baseMicros + microsOffset;
    uint32_t hours = totalMicros / 3600000000;
    uint32_t mins = (totalMicros % 3600000000) / 60000000;
    uint32_t secs = (totalMicros % 60000000) / 1000000;
    uint32_t micros = totalMicros % 1000000;
    char buf[24];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%06d", hours, mins, secs, micros);
    return String(buf);
}

void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    uint32_t receivedMicros = micros();
    static uint32_t prevMicros = 0;
    if (receivedMicros < prevMicros) baseMicros += 0xFFFFFFFF;
    prevMicros = receivedMicros;

    uint16_t samples = length / 6; // 6 bytes/sample
    for (int i = 0; i < samples; i++) {
        int16_t xRaw, yRaw, zRaw;
        memcpy(&xRaw, pData + i * 6, 2);
        memcpy(&yRaw, pData + i * 6 + 2, 2);
        memcpy(&zRaw, pData + i * 6 + 4, 2);

        float x = xRaw / 8192.0;
        float y = yRaw / 8192.0;
        float z = zRaw / 8192.0;

        String ts = formatTimestamp(receivedMicros);
        Serial.printf("[%s] X:%.3f Y:%.3f Z:%.3f\n", ts.c_str(), x, y, z);
    }

    totalSamples += samples;
    uint32_t elapsed = receivedMicros - lastReportMicros;
    if (elapsed >= 1000000) {
        float rate = (totalSamples * 1000000.0) / elapsed;
        Serial.printf("\nRate: %.1f samples/sec\n\n", rate);
        totalSamples = 0;
        lastReportMicros = receivedMicros;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 IMU Receiver");

    BLEDevice::init("IMU_Receiver");
    BLEDevice::setMTU(247);

    BLEScan* pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    BLEScanResults results = *pScan->start(5);

    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        if (device.isAdvertisingService(serviceUUID)) {
            BLEClient* pClient = BLEDevice::createClient();
            if (pClient->connect(&device)) {
                BLERemoteService* pService = pClient->getService(serviceUUID);
                if (pService) {
                    BLERemoteCharacteristic* pChar = pService->getCharacteristic(charUUID);
                    if (pChar->canNotify()) {
                        pChar->registerForNotify(notifyCallback);
                        pChar->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t[]){0x01,0x00}, 2, true);
                        Serial.println("Notifications enabled!");
                    }
                }
            }
        }
    }
}

void loop() {
    delay(1000);
}
