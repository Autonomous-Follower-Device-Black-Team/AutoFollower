#include <Arduino.h>
#include "config.h"
#include "Device.h"

SocConfig soc = SocConfig::NONE;
Device belt(soc);

void setup() {
    Serial.begin(BAUD_RATE);    
    Serial.println("Entering Belt Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }

    //belt.init();
    Serial.print(TaskStackDepth::tsd_DRIVE);
    belt.getPeripheralManager()->initPeripherals();
}

bool success = false;
float distance = -1.0;
void loop() {
    
    success = belt.getPeripheralManager()->fetchUS(SensorID::transducer)->readSensor(US_READ_TIME);
    if(success) {
        distance = belt.getPeripheralManager()->fetchUS(SensorID::transducer)->getDistanceReading();
        Serial.printf("Distance: %f\n", distance);
    }
    else {
        Serial.println("Reading unsuccesful.");
    }
    vTaskDelay(1000);
}