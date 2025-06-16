#include "Device.h"

TaskHandle_t ping_timer_task_handle = NULL;

void ping_timer_task(void *pvParams) {
    // Setup.
    Device *device = static_cast<Device *>(pvParams);

    // Loop.
    for(;;) {
        // Wait for notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        // Begin delay 5s.
        delay(5000);

        // Raise Pin High and Hold for 1s.
        device->trigger();
        
    }
}

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

void Device::trigger() {
    digitalWrite(trig_pin, HIGH);
    delay(1000);
    digitalWrite(trig_pin, LOW);
}