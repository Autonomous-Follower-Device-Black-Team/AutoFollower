#include <Arduino.h>

#include "Config.h"
#include "EspNowNode.h"
#include "Device.h"

Device bot;

void setup() {
    Serial.begin(BAUD_RATE);    
    Serial.println("Entering Bot Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }

    bot.init();
}

void loop() {

}