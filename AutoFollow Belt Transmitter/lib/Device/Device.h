
#ifndef DEVICE_H
#define DEVICE_H

#include "EspNowNode.h"

class Device {
    private:
        EspNowNode tx;
        static BaseType_t processHandshake(const char* data);
        static BaseType_t processWave(const char* data);
        static BaseType_t processInfoReceived(const char* data);

    public:
        Device() : tx(dev_S3_A, Mode::Transmitter) {};
        void init();

};

#endif /* Device.h */