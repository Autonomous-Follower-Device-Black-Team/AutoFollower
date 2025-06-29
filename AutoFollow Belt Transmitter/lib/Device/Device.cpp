#include "Device.h"

esp_timer_handle_t trigger_timer_handle = NULL;
TaskHandle_t ping_timer_task_handle = NULL;

void trigger_timer_callback(void *arg) {
    Device *dev = static_cast<Device *>(arg);
    dev->setTriggerTimerFlag(true);
    dev->timer_on = false;
    if(dev->isTransmitter()) {
        xTaskNotify(trig_tx_transducer_task_handle, -1, eNoAction);
    }
    else {
        xTaskNotify(trig_left_rx_transducer_task_handle, -1, eNoAction);
        xTaskNotify(trig_right_rx_transducer_task_handle, -1, eNoAction);
    }
}

void ping_timer_task(void *pvParams) {
    // Setup.
    Device *device = static_cast<Device *>(pvParams);

    // Loop.
    for(;;) {
        // Wait for notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        if(device->getTriggerTimerFlag() == true) {
            device->toggleRgbLed();
            device->setTriggerTimerFlag(false);
            Serial.println("toggled in ping task");
        }

        //vTaskDelay(pdMS_TO_TICKS(100));
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
    // Notify the timer tasks if the device is the 
    if(!deviceIsTx) 
        if(trigger_timer_handle == NULL) log_e("Trigger Timer Not Created.");
        else {
            if(!timer_on) startOneshotEspTimer(triggerTimerDelay);
        }
    return pdPASS;
}

BaseType_t Device::processDataSent(const char* data) {
    if(deviceIsTx) 
        if(trigger_timer_handle == NULL) log_e("Trigger Timer Not Created.");
        else {
            if(!timer_on) startOneshotEspTimer(triggerTimerDelay);
        }
    return pdPASS;
}

void Device::init() {
    startPeripheralManager();
    startESPNow();
    startDriveSystem();
}

void Device::startPeripheralManager()  {
    manager->beginTasks();
    manager->initUS();
    manager->attachInterrupts();
}

void Device::startESPNow() {
    tx->registerProcessHandshakeCallBack(Device::processHandshake);
    tx->registerProcessWaveCallBack(Device::processWave);
    tx->registerProcessInfoReceivedCallBack(Device::processInfoReceived);
    tx->registerDataSentCallBack(Device::processDataSent);
    tx->start();
}

void Device::startDriveSystem() {

}

void Device::createOneshotEspTimer(uint64_t delay) {
    triggerTimerDelay = delay;
    esp_err_t success = esp_timer_create(
        &trigger_timer_params, 
        &trigger_timer_handle
    );
    if(success  != ESP_OK) log_e("esp_timer unable to be created.");
    else log_e("Created timer.");
}

/**
 * @param delay Hardware timer delay in milliseconds.
 */
esp_err_t Device::startOneshotEspTimer(uint64_t delay) {
    timer_on = true;
    esp_err_t success = delay *= 1000;
    success = esp_timer_start_once(
        trigger_timer_handle, 
        delay
    );
    if(success  != ESP_OK) log_e("oneshot esp_timer unable to start.");
    //else log_e("Started oneshot timer.");
    return success;
}

void Device::toggleRgbLed() {
    if(!ledOn) rgbLedWrite((int) S3BeltPin::rgbLed, RGB_BRIGHTNESS, 0, 0);
    else rgbLedWrite((int) S3BeltPin::rgbLed, 0, 0, 0);
    ledOn ^= 1;
}

bool Device::isTransmitter() { return tx->isNodeTransmitter();}

BaseType_t Device::beginPingTimerTask() {
    return xTaskCreatePinnedToCore(
        &ping_timer_task,           // Pointer to task function.
        "ping_timer_task",          // Task name.
        4096,                       // Size of stack allocated to the task (in bytes).
        this,                       // Pointer to parameters used for task creation.
        1,                          // Task priority level.
        &ping_timer_task_handle,    // Pointer to task handle.
        1                           // Core that the task will run on.
    );
}

void Device::initTasks() {
    BaseType_t res = pdFAIL;

    // Begin the communication task.
    res = beginPingTimerTask();
    if(res != pdPASS) Serial.println("Ping Timer Task Not Started!");
    else Serial.println("Ping Timer Task Started Succesfully!");
}

void Device::testTriggerSynchronization() {
    // Init everything.
    /*
    manager->initUS();
    manager->attachInterrupts();
    manager->beginTasks();
    startESPNow();
    */

    createOneshotEspTimer();
    beginPingTimerTask();
}