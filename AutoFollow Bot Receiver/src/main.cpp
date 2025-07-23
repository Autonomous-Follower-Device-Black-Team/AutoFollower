#include <Arduino.h>

#include "Config.h"
#include "Device.h"

void indicateSetup();

char info[1000]; 
bool success = false, listPrinted = false;
Device autoFollowerDevice(SocConfig::ESP32_S3_8MB, dev_C, Mode::Receiver, true);

void setup() {
    // Start Serial.
    Serial.begin(BAUD_RATE);  
    indicateSetup();

    // Initialize robot.
    autoFollowerDevice.init();
}

void loop() {
    if(!listPrinted) {
        vTaskList(info);
        Serial.println(info);
        listPrinted = true;
    }
    vTaskDelay(10000);
}

void indicateSetup() {
    log_e("Beginning Bot Setup.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
}

