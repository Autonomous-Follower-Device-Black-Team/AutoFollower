#include "Device.h"

BaseType_t Device::processHandshake(const char* data) {
    Serial.println("Handshake Received.");
    return pdPASS;
}

BaseType_t Device::processWave(const char* data) {
    Serial.println("Wave Received.");
    return pdPASS;
}

BaseType_t Device::processInfoReceived(const char* data) {
    //Serial.printf("[%lu] Data Received: %s\n", millis(), data);
    return pdPASS;
}

void Device::init() {
    tx.registerProcessHandshakeCallBack(Device::processHandshake);
    tx.registerProcessWaveCallBack(Device::processWave);
    tx.registerProcessInfoReceivedCallBack(Device::processInfoReceived);
    tx.start();
}
