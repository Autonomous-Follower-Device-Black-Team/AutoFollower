#include <Arduino.h>

#include "beltConfig.h"
#include "EspNowNode.h"
#include "Device.h"

Device belt;

void setup() {
    Serial.begin(BAUD_RATE);    
    Serial.println("Entering Belt Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }

    belt.init();
}

void loop() {

}