
#ifndef DEVICE_H
#define DEVICE_H

#include "config.h"
#include "EspNowNode/EspNowNode.h"
#include "PeripheralManager/PeripheralManager.h"

extern esp_timer_handle_t trigger_timer_handle;
void trigger_timer_callback(void *arg);

extern TaskHandle_t ping_timer_task_handle;
void ping_timer_task(void *pvParams);

class Device {
    private:

        inline static bool deviceIsTx = true;
        bool ledOn = false;
        bool trigger_timer_flag = false;
        inline static uint64_t triggerTimerDelay = -1;
        const esp_timer_create_args_t trigger_timer_params = {
            .callback = &trigger_timer_callback,
            .arg = this, 
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ESP TIMER",
            .skip_unhandled_events = true
        };

        SocConfig socInUse;
        PeripheralManager *manager;
        EspNowNode *tx;

        static BaseType_t processHandshake(const char* data);
        static BaseType_t processWave(const char* data);
        static BaseType_t processInfoReceived(const char* data);
        static BaseType_t processDataSent(const char* data);

        void initTasks();

    public:
        Device(SocConfig soc, const uint8_t* peerMacAddress, Mode mode, bool ackRequired) {
            // Grab the SoC.
            this->socInUse = soc;
            
            // Create the ESP Now Node and Manager.
            this->tx = new EspNowNode(peerMacAddress, mode, ackRequired);
            this->manager = new PeripheralManager(this);

            // Do some checks.
            deviceIsTx = (mode == Mode::Transmitter) ? true : false;
            if(soc == SocConfig::ESP32_S3_8MB) {
                pinMode((int) S3BeltPin::rgbLed, OUTPUT);
                log_e("RGB LED Pin set as Ouput.");
            }
        }

        void init();
                
        void startPeripheralManager();
        void startESPNow();
        void startDriveSystem();

        void setTriggerTimerFlag(bool val) { trigger_timer_flag = true; }
        bool getTriggerTimerFlag() { return trigger_timer_flag; }
        
        inline static bool timer_on = false;
        void toggleRgbLed();
        void createOneshotEspTimer(uint64_t delay = 1);
        static esp_err_t startOneshotEspTimer(uint64_t delay = 1);
        BaseType_t beginPingTimerTask();
        
        bool isTransmitter();
        PeripheralManager *getPeripheralManager() { return manager; }
        SocConfig getSocInUse() { return socInUse; }

        void testTriggerSynchronization();
};

#endif /* Device.h */