
#ifndef DEVICE_H
#define DEVICE_H

#include "config.h"
#include "EspNowNode/EspNowNode.h"
#include "PeripheralManager/PeripheralManager.h"

extern TaskHandle_t ping_timer_task_handle;
void ping_timer_task(void *pvParams);

class Device {
    private:
        SocConfig socInUse;
        PeripheralManager *manager;
        static EspNowNode tx;

        static BaseType_t processHandshake(const char* data);
        static BaseType_t processWave(const char* data);
        static BaseType_t processInfoReceived(const char* data);
        static BaseType_t dataSent(const char* data);


        BaseType_t beginPingTimerTask();
        void initTasks();

    public:
        Device(SocConfig soc) : 
            socInUse(soc), 
            manager(new PeripheralManager(this)) 
        {};
        void init();
        static bool isTransmitter();
        PeripheralManager *getPeripheralManager() { return manager; }
        SocConfig getSocInUse() { return socInUse; }

};

#endif /* Device.h */