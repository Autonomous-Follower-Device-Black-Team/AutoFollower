
// Include gaurd.
#ifndef MOTOR_H
#define MOTOR_H

#include <esp32-hal-ledc.h>
#include "driver/mcpwm_prelude.h"
#include <Arduino.h>

#define PWM_FREQ 20000
#define PWM_RES 8
#define LED_C_LOW 0
#define LED_C_HIGH 255

enum stopType {
    COAST, 
    BRAKE
};


class Motor {
    
    private:
        const int posTerm;          // PWM 1 terminal.
        const int negTerm;          // PWM 2 terminal

        int currentMonitor = -1;
        int diagnostic = -1;
        
        int speed = 0;              // Speed motor should be driving in.
        float currLim = 2000.0;     // Current limit through motor in milli-Amps.
        bool enabled = false;       // Motor enable pin status.
        bool isSwitching = false;   // Motor direction switching status for cases where the motor stops but experiences overcurrent.
        ulong onTime = 0;           // Time (in ms) that the motor turned on from rest.

    public:
        Motor(int posTerm, int negTerm) : 
            posTerm(posTerm), 
            negTerm(negTerm) {};

        void init();
        void spinCW();
        void spinCCW();
        void setSpeed(int speed);
        int getSpeed();
        bool monitorOverCurrentConditions();
        bool monitorDiagnosticConditions();
        bool getSwitchingDirectionStatus();
        ulong getOnTime();
        void stop(stopType sType);

        bool getEnableStatus();
        //bool getRotationStatus();
};

// End include gaurd.
#endif /* Motor.h */