
#ifndef DEVICE_H
#define DEVICE_H

#include "EspNowNode.h"

class Device {
    private:
        EspNowNode tx;
        static BaseType_t processHandshake(const char* data);
        static BaseType_t processWave(const char* data);
        static BaseType_t processInfoReceived(const char* data);
        uint8_t trig_pin = 5;

    public:
        Device() : tx(dev_S3_A, Mode::Transmitter) {
            pinMode(trig_pin, OUTPUT);
            digitalWrite(trig_pin, LOW);
        };
        void init();
        void trigger();

};

#endif /* Device.h */