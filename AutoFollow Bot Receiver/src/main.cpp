#include <Arduino.h>

#include "Config.h"
#include "Device.h"

///*
void setupWifi();
String getTimeDiff(bool end);

Device bot(SocConfig::ESP32_S3_8MB, dev_S3_A, Mode::Receiver, true);

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

bool listPrinted = true;
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

void setupWifi() {
   
    WiFi.begin(af_SSID, af_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }
}

String getTimeDiff(bool end) {
    PeripheralManager *m = bot.getPeripheralManager();
    ulong diff = 0;
    if(xSemaphoreTake(rx_echo_time_mutex, portMAX_DELAY) == pdTRUE) {
        USTimeGroup *g = m->getUsTimingGroup();
        if(end) diff =  g->rightEndTime - g->leftEndTime;
        else diff =  g->rightStartTime - g->leftStartTime;
        xSemaphoreGive(rx_echo_time_mutex);
    }
    return String(diff);
}

//*/

