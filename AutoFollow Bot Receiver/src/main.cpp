#include <Arduino.h>

#include "Config.h"
#include "Device.h"

Device bot(SocConfig::ESP32_S3_8MB, dev_C, Mode::Receiver, true);

bool success = false;
char info[1000]; 

void setup() {
    Serial.begin(BAUD_RATE);   
    log_e("Entering Bot Setup.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }

    //setupWifi(); 
    
    bot.createOneshotEspTimer(TTR_US);
    bot.startPeripheralManager();   
    bot.startESPNow(WiFi.status() == WL_CONNECTED);

    log_e("Bot Setup Complete.");
}

bool listPrinted = false;
void loop() {
    if(!listPrinted) {
        vTaskList(info);
        Serial.println(info);
        listPrinted = true;
    }
    else {
        //Serial.println("Looping");
    }
    vTaskDelay(10000);
}
