#include <Arduino.h>

#include "Config.h"
#include "Device.h"

Device bot(SocConfig::ESP32_S3_8MB, dev_S3_A, Mode::Receiver, true);

bool success = false;
char info[1000]; 

void setup() {
    Serial.begin(BAUD_RATE);   
    delay(1000); 
    log_e("Entering Bot Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }

    bot.createOneshotEspTimer(TTR_US);
    bot.startPeripheralManager();   
    bot.startESPNow();

    log_e("Bot Setup Complete.");
}

bool listPrinted = false;
void loop() {
    if(!listPrinted) {
        vTaskList(info);
        Serial.println(info);
        listPrinted = true;
    }
    vTaskDelay(1000);
}