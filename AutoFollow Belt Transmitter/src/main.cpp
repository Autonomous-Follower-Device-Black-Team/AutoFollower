#include <Arduino.h>
#include "config.h"
#include "Device.h"

SocConfig soc = SocConfig::ESP32_S3_8MB;
Device belt(SocConfig::ESP32_S3_8MB, dev_S3_B, Mode::Transmitter, true);

bool success = false;
char info[1000]; 

void setup() {
    Serial.begin(BAUD_RATE);  
    delay(1000);
    log_e("Entering Belt Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }

    belt.createOneshotEspTimer(TTR_US - 2);
    belt.startPeripheralManager();
    //belt.beginPingTimerTask();
    belt.startESPNow();

    log_e("Belt Setup Complete.");
}

void loop() {
    //vTaskList(info);
    //Serial.println(info);
    vTaskDelay(1000);
}

