#include <Arduino.h>

#include "Config.h"
#include "Device.h"

/*
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

#define _SSID "SEEMS"
#define _PASS "@Ucf2025"

const char* PARAM_MESSAGE = "message";

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setup() {

    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(_SSID, _PASS);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message);
    });

    // Send a POST request to <IP>/post with a form field message set to <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
    });

    server.onNotFound(notFound);

    server.begin();
}

void loop() {
}
*/

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

/*
TaskHandle_t trig_handle = NULL;
TaskHandle_t left_handle = NULL;
TaskHandle_t right_handle = NULL;
TaskHandle_t diff_handle = NULL;

void trig_task(void *args);
void left_task(void *args);
void right_task(void *args);
void diff_task(void *args);
void createTask(TaskFunction_t func, TaskHandle_t *handle, const char *name);

void setup() {
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    Serial.begin(BAUD_RATE);
    createTask(trig_task, &trig_handle, "trig");
    createTask(left_task, &left_handle, "left");
    createTask(right_task, &right_handle, "right");
    createTask(diff_task, &diff_handle, "diff");
}

void loop() {
    // Do Nothing.
}

void createTask(TaskFunction_t func, TaskHandle_t *handle, const char *name) {
    xTaskCreatePinnedToCore(
        func,           // Pointer to task function.
        name,           // Task name.
        4096,           // Size of stack allocated to the task (in bytes).
        NULL,           // Pointer to parameters used for task creation.
        1,              // Task priority level.
        handle,        // Pointer to task handle.
        1               // Core that the task will run on.
    );
}

void trig_task(void *args) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TaskHandle_t handle = xTaskGetCurrentTaskHandle();
    if(handle == NULL) log_e("Handle was Null.");
    else log_e("All Good");
    int delay = pdMS_TO_TICKS(1000);
    
    for(;;) {
        int val = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1));
        //log_e("Trigger! %d", val);

        if(left_handle != NULL) xTaskNotify(left_handle, -1, eNoAction);
        else log_e("Left Handle Null");

        if(right_handle != NULL) xTaskNotify(right_handle, -1, eNoAction);
        else log_e("Right Handle Null");

        vTaskDelayUntil(&xLastWakeTime, delay);
    }
}

void left_task(void *args) {
    int i = 0;
    String name = "left_task";
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
        Serial.printf("[%d] %s notified\n", i++, name.c_str());

        if(diff_handle != NULL) xTaskNotify(diff_handle, -1, eIncrement);
        else log_e("Diff Handle Null");
    }
}

void right_task(void *args) {
    int i = 0;
    String name = "right_task";
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
        Serial.printf("[%d] %s notified\n", i++, name.c_str());

        if(diff_handle != NULL) xTaskNotify(diff_handle, -1, eIncrement);
        else log_e("Diff Handle Null");
    }
}
 
void diff_task(void *args) {
    int i = 0;
    uint32_t notifVal = -1;
    String name = "diff_task";
    for(;;) {
        xTaskNotifyWait(0, 0, &notifVal, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5000));
        if(notifVal == 2) {
            Serial.printf("[%d] %s notified\n", i++, name.c_str());
            ulTaskNotifyValueClear(NULL, UINT_MAX);
        }
        else log_e("Diff awake. Notif Value: %d", notifVal);
    }   
}

//*/