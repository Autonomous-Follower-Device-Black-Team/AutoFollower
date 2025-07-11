#include <Arduino.h>
#include "config.h"
#include "Device.h"

void startWifi();

SocConfig soc = SocConfig::ESP32_S3_8MB;
Device belt(SocConfig::ESP32_S3_8MB, dev_S3_AFD_RX, Mode::Transmitter, true);

bool success = false;
char info[1000]; 

void setup() {
    Serial.begin(BAUD_RATE);  
    log_e("Entering Belt Setup.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }

    //startWifi();
    
    belt.createOneshotEspTimer(TTR_US);
    belt.startPeripheralManager();
    belt.startESPNow(WiFi.status() == WL_CONNECTED);

    log_e("Belt Setup Complete.");
}

bool listShown = true;
void loop() {
    if(listShown) {
        vTaskList(info);
        Serial.println(info); 
        listShown = false;
    }
    vTaskDelay(1000);
}

void startWifi() {
    WiFi.begin(af_SSID, af_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }
}