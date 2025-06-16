#include "Device.h"

TaskHandle_t ping_timer_task_handle = NULL;
EspNowNode Device::tx(dev_C, Mode::Transmitter, true); 

void ping_timer_task(void *pvParams) {
    // Setup.
    Device *device = static_cast<Device *>(pvParams);

    // Loop.
    for(;;) {
        // Wait for notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        // Begin delay 5s.
        delay(5000);

        
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
    if(isTransmitter() == false) 
        xTaskNotify(ping_timer_task_handle, -1, eNoAction);
    return pdPASS;
}

BaseType_t Device::dataSent(const char* data) {
    if(isTransmitter()) 
        xTaskNotify(ping_timer_task_handle, -1, eNoAction);
    return pdPASS;
}

void Device::init() {
    manager->initPeripherals();
    
    tx.registerProcessHandshakeCallBack(Device::processHandshake);
    tx.registerProcessWaveCallBack(Device::processWave);
    tx.registerProcessInfoReceivedCallBack(Device::processInfoReceived);
    tx.start();
}

bool Device::isTransmitter() { return tx.isNodeTransmitter();}

BaseType_t Device::beginPingTimerTask() {
    return xTaskCreatePinnedToCore(
        &esp_now_tx_rx_task,   // Pointer to task function.
        "communication_task",  // Task name.
        ESPNOW_TASK_DEPTH,       // Size of stack allocated to the task (in bytes).
        this,                   // Pointer to parameters used for task creation.
        1,                      // Task priority level.
        &esp_now_tx_rx_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );
}

void Device::initTasks() {
    BaseType_t res = pdFAIL;

    // Begin the communication task.
    res = beginPingTimerTask();
    if(res != pdPASS) Serial.println("Ping Timer Task Not Started!");
    else Serial.println("Ping Timer Task Started Succesfully!");
}