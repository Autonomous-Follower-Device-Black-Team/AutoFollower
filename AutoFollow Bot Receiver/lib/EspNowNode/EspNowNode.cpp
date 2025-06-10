#include "EspNowNode.h"
#include <Arduino.h>
#include <ESP32_NOW.h>
#include <WiFi.h>
#include <esp_mac.h>

// Define task handles.
TaskHandle_t esp_now_tx_rx_handle = NULL;
TaskHandle_t esp_now_process_data_handle = NULL;

void esp_now_tx_rx_task(void *pvParams) {
    // Setup.
    EspNowNode *node = static_cast<EspNowNode *>(pvParams);
    int msg_count = 0;
    String str;
    int bufferSize = 100;
    char buffer[bufferSize];
    bool success = false;

    // Task loop.
    for(;;) {

        // End if possible.
        if(node->isTransmissionPaused()) {
            node->end();
        }

        // Ready to transmit.
        if(node->readyToTransmit() == true && !node->isTransmissionPaused()) {

            //node->reRegister();
            success = node->transmit();
            if(success == false) {
                Serial.println("Failed Transmission");
                node->reRegister();
            }
        }
        
        // Ready to receive.
        else {
            if(node->isNodeTransmitter()) Serial.println("Waiting for Acknowledgment From Receiver (Bot)");
            else Serial.println("Waiting for acknowledgement from Transmitter (Belt)");
        }
        vTaskDelay(pdMS_TO_TICKS(TaskDelayLength));
    }
}

void esp_now_process_data_task(void *pvParams) {
    // Setup.
    EspNowNode *node = static_cast<EspNowNode *>(pvParams);
    bool success = false;

    // Task loop.
    for(;;) {
        // Wait for notifcation before processing data.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  
        
        // End if possible.
        if(node->isTransmissionPaused()) {
            node->end();
        }
        
        // Call the proper call back based on data sent.
        //log_e("processing data called");
        node->showDataReceived();
        success = node->proccessPacket();
        if(success) node->setReadyToTransmit(true);
        //else log_e("Data processing failed.");
        
        // No need to delay due to blocking by notifcation waiting.
    }
}

bool EspNowNode::send_message() {
    bool res = true;
    if(!send((uint8_t *) &outgoingData, sizeof(ESP_NOW_PACKET))) {
        //log_e("Failed to broadcast message!");
        res = false;
    }
    return res;
}

void EspNowNode::initWifi() {
    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    while(!WiFi.STA.started()) vTaskDelay(pdMS_TO_TICKS(100));
}

void EspNowNode::initESPNOW() {

    // Begin ESP NOW and add Peer to the network.
    bool success = true;
    if(!ESP_NOW.begin()) {
        //log_e("Failed to init ESP-NOW!");
        success = false;
    }
    if(!success || !add()) {
        //log_e("Failed to register broadcast peer!");
        success = false;
    } 

    if(!success) {
        Serial.println("Failed to initilize ESP NOW.");
        Serial.println("Rebooting");
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP.restart();
    }
    esp_now_setup = true;
    Serial.println("Peer Has Begun Broadcasting");
    Serial.println("Communication info:");
    Serial.println("\t Mode: " + String(WiFi.getMode()));
    Serial.println("\t Node Mac Address: " + getThisMacAddress());
    Serial.println("\t Peer Mac Address: " + getPeerMacAddress());
    Serial.println("\t Channel: " + String(WiFi.channel()));
}

void EspNowNode::initTasks() {
    BaseType_t res = pdFAIL;

    // Begin the communication task.
    res = beginCommunicationTask();
    if(res != pdPASS) Serial.println("ESP Now Communication Task Not Started!");
    else Serial.println("ESP Now Communication Task Started Succesfully!");

    // Begin data processing task.
    res = beginProcessDataTask();
    if(res != pdPASS) Serial.println("ESP Now Process Data Task Not Started!");
    else Serial.println("ESP Now Process Data Task Started Succesfully!");
}

BaseType_t EspNowNode::beginCommunicationTask() {
    return xTaskCreatePinnedToCore(
        &esp_now_tx_rx_task,   // Pointer to task function.
        "communication_task",  // Task name.
        ESPNOW_TASK_DEPTH,       // Size of stack allocated to the task (in bytes).
        this,                   // Pointer to parameters used for task creation.
        1,           // Task priority level.
        &esp_now_tx_rx_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );
}

BaseType_t EspNowNode::beginProcessDataTask() {
    return xTaskCreatePinnedToCore(
        &esp_now_process_data_task,     // Pointer to task function.
        "process_Data_task",            // Task name.
        ESPNOW_TASK_DEPTH,              // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        1,                              // Task priority level.
        &esp_now_process_data_handle,   // Pointer to task handle.
        1                               // Core that the task will run on.
    );
}

void EspNowNode::buildTransmission() {
    // Determine the next packet components.
    Header head = determineNextHeader();
    AckMessage ack = determineNextAck();
    String data = determineNextData();

    // Construct the next packet.
    outgoingData.header = head;
    outgoingData.ack = ack;
    sprintf(outgoingData.data, "%s", data.c_str());
}

bool EspNowNode::registerProcessHandshakeCallBack(ProcessDataCallback pcb) {
    handshakeCallback = pcb;
    return true;
}

bool EspNowNode::registerProcessWaveCallBack(ProcessDataCallback pcb) {
    waveCallback = pcb;
    return true;
}

bool EspNowNode::registerProcessInfoReceivedCallBack(ProcessDataCallback pcb) {
    infoReceivedCallback = pcb;
    return true;
}   

bool EspNowNode::start() {

    // Ensure proper callbacks are registered.
    bool success = false;
    success = (handshakeCallback != NULL && waveCallback != NULL && infoReceivedCallback != NULL);
    
    // Only start if callbacks are all good.
    if(success) {
        initWifi();
        initESPNOW();
        initTasks();
    }
    else log_e("ESP Now Not started. Ensure all callbacks are registred");
    
    // Return.
    return success;
}

bool EspNowNode::end() {
    bool res = true;

    // Delete tasks to free up the scheduler.
    vTaskDelete(esp_now_tx_rx_handle);
    vTaskDelete(esp_now_process_data_handle);
    esp_now_tx_rx_handle = NULL;
    esp_now_process_data_handle = NULL;

    // Deinitialize ESP NOW.
    this->remove();
    esp_now_deinit();

    // Return.
    return res;
}

void EspNowNode::pause() { isPaused = true; }

void EspNowNode::unpause() { isPaused = false; }

bool EspNowNode::isTransmissionPaused() { return isPaused; }

void EspNowNode::onReceive(const uint8_t *data, size_t len, bool broadcast) {

    // Save data received.
    ESP_NOW_PACKET *dataReceived = (ESP_NOW_PACKET *) data;
    incomingData.header = dataReceived->header;
    incomingData.ack = dataReceived->ack;
    sprintf(incomingData.data, "%s", dataReceived->data);

    // Notify the process Data task.
    xTaskNotify(esp_now_process_data_handle, -1, eNoAction);
}

void EspNowNode::onSent(bool success) {
    if(ackRequired) this->waitingForData = true;
}

bool EspNowNode::is_esp_now_setup() { return esp_now_setup; }

bool EspNowNode::isNodeTransmitter() { return mode == Mode::Transmitter;}

bool EspNowNode::transmit() { 
    // Construct the transmission and send it.
    buildTransmission();
    showDataTransmitted();
    return send_message(); 
}

bool EspNowNode::readyToTransmit() { return !waitingForData; }

Header EspNowNode::getHeaderToProcess() { return incomingData.header; }

const char* EspNowNode::getDataToProcess() { return incomingData.data; }

void EspNowNode::setReadyToTransmit(bool status) {
    waitingForData = !status;
}

void EspNowNode::showDataReceived() {
    Serial.println("------------------------------------------------");
    Serial.printf("Synced Time: %lld\n", esp_wifi_get_tsf_time(WIFI_IF_AP));
    Serial.printf("System Time: %lu\n", micros());
    Serial.printf("WiFi Channel: %d\n",  WiFi.channel());
    Serial.printf("Header Received: %d\n", incomingData.header);
    Serial.printf("Ack Msg Received: %c\n", incomingData.ack);
    Serial.printf("Data Received: %s\n", incomingData.data);
    Serial.println("------------------------------------------------\n");
}

void EspNowNode::showDataTransmitted() {
    Serial.println("------------------------------------------------");
    Serial.printf("Synced Time: %lld\n", esp_wifi_get_tsf_time(WIFI_IF_AP));
    Serial.printf("System Time: %lu\n", micros());
    Serial.printf("WiFi Channel: %d\n",  WiFi.channel());
    Serial.printf("Header Transmitted: %d\n", outgoingData.header);
    Serial.printf("Ack Msg Transmitted: %c\n", outgoingData.ack);
    Serial.printf("Data Transmitted: %s\n", outgoingData.data);
    Serial.println("------------------------------------------------\n");
}

bool EspNowNode::proccessPacket() {
    // Retreive data to deal with.
    BaseType_t res = pdFAIL;
    Header headerToProcess = getHeaderToProcess();
    const char* dataToProcess = getDataToProcess();
    
    switch (headerToProcess) {
        // Process Handshake.
        case Header::HANDSHAKE :
            if(handshakeCallback != NULL) handshakeCallback(dataToProcess);
            else Serial.println("Unable to process handshake. Handshake Processing Callback Not Assigned.");
            break;
            
        // Process Wave.
        case Header::WAVE :
            if(waveCallback != NULL) waveCallback(dataToProcess);
            else Serial.println("Unable to process wave. Wave Processing Callback Not Assigned.");
            break;
            
        // Process Acknow
        case Header::TRIGGER_PING:
            if(infoReceivedCallback != NULL) infoReceivedCallback(dataToProcess);
            else Serial.println("Unable to process ping. InfoReceived Processing Callback Not Assigned.");
            break;

        default:
            Serial.printf("Unable to Process Info (Unknown Header: %d)\n", headerToProcess);
            break;
    }
    // Clear the waiting for data flag and return.
    waitingForData = false;
    return (res == pdPASS);
}

void EspNowNode::reRegisterPeer() {
    bool removed = this->remove();
    //log_e("Peer remove status: %s", removed ? "success" : "failed");

    this->setChannel(WiFi.channel());

    bool added = this->add();
    //log_e("Peer add status: %s", added ? "success" : "failed");
}

// Only for Master.
Header EspNowNode::determineNextHeader() {

    Header headerReceived = incomingData.header;
    Header nextHeaderToSend = Header::HANDSHAKE;
    
    // Choose next header based on last header if node didn't just start.
    if(!justStarted) {
        switch(headerReceived) {
            
            // Send Ping after initial connection.
            case Header::HANDSHAKE : 
                nextHeaderToSend = Header::TRIGGER_PING;
                break;
            
            // Time to disconnect.
            case Header::WAVE :
                nextHeaderToSend = Header::HANDSHAKE;
                isPaused = true;
                break;
            
            // Send ping after each ping.
            case Header::TRIGGER_PING : 
                nextHeaderToSend = Header::TRIGGER_PING;
                break;
    
            default:
                Serial.println("Unable to determine next header to transmit.");
                break;
        }
    }

    // Clear the just started flag for future pass throuhghs.
    else justStarted = false;

    // Return;
    return nextHeaderToSend;
}

// Only for Slave.
AckMessage EspNowNode::determineNextAck() {
    Header prevHeaderReceived = incomingData.header;
    AckMessage nextAckToSend = AckMessage::Received_Handshake;
    
    // Choose next ack based on last header if node didn't just start.
    if(!justStarted) {
        if(isNodeTransmitter()) nextAckToSend = AckMessage::Received_Handshake;
        switch(prevHeaderReceived) {
            
            // Acknowledge receipt of handshake.
            case Header::HANDSHAKE : 
                nextAckToSend = AckMessage::Received_Handshake;
                break;
            
            // Acknowledge receipt of wave.
            case Header::WAVE :
                nextAckToSend = AckMessage::Received_Wave;
                isPaused = true;
                break;
            
            // Acknowledge receipt of ping.
            case Header::TRIGGER_PING : 
                nextAckToSend = AckMessage::Received_Ping;
                break;
                
            default:
                Serial.println("Unable to determine next ack to transmit.");
                break;
        }
    }

    // Clear the just started flag for future pass throuhghs.
    else justStarted = false;

    // Return;
    return nextAckToSend; 
}

String EspNowNode::determineNextData() {
    
    // Send current system time.
    String nextDataToSend = String(millis());

    // Return;
    return nextDataToSend;
}

void EspNowNode::reRegister() { reRegisterPeer(); }

String EspNowNode::getPeerMacAddress() {
    char macStr[18] = {0};
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
                peerMacAddress[0],
                peerMacAddress[1], 
                peerMacAddress[2], 
                peerMacAddress[3], 
                peerMacAddress[4], 
                peerMacAddress[5]
            );
    return String(macStr);
}

String EspNowNode::getThisMacAddress() {
    return String(WiFi.macAddress());
}