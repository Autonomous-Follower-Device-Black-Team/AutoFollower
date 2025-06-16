#include "Motor.h"

void Motor::init() {
    // Set motor terminals as PWM outputs.
    ledcAttach(posTerm, PWM_FREQ, PWM_RES);
    ledcAttach(negTerm, PWM_FREQ, PWM_RES);
}

void Motor::spinCW() {
    if(enabled) {
        onTime = millis();
        if(isSwitching == true) isSwitching = false;
        ledcWrite(posTerm, LED_C_LOW);
        ledcWrite(negTerm, speed);
        }
}

void Motor::spinCCW() {
    if(enabled) {
        onTime = millis();
        if(isSwitching == true) isSwitching = false;
        ledcWrite(posTerm, speed);
        ledcWrite(negTerm, LED_C_LOW);
    }
}

void Motor::setSpeed(int speed) {
    this->speed = speed;
}

int Motor::getSpeed() {
    return this->speed;
}

bool Motor::monitorOverCurrentConditions() {

    // Only run on active motors. 
    bool res = false;
    if(!enabled) res = false;
    else {
        float value = analogReadMilliVolts(currentMonitor);
        float comparisonCurrent = value / 500.0 * 1000;   // 500 mV per Amp.

        if(comparisonCurrent >= currLim) {
            res = true;
            Serial.printf("------\tOCM Pin Read = %f mV, Current = %f mA,[%f >= %f]\n", value, comparisonCurrent, comparisonCurrent, currLim);
            Serial.printf("------\tCooling Down\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else {
            res = false;
            //Serial.printf("\tOCM Pin Read = %f mV, Current = %f mA, [%f < %f]\n", value, comparisonCurrent, comparisonCurrent, currLim);
        }

    }

    // Return.
    return res;
}

/**
 * Read the motors diagnostic pin.
 * @return True when motor diagnostic pin is driven low, indicating an error condition.
 */
bool Motor::monitorDiagnosticConditions() {
    int diagLvl = digitalRead(diagnostic);
    return (diagLvl == LOW) ? true : false;
}

bool Motor::getSwitchingDirectionStatus() {
    return isSwitching;
}

void Motor::stop(stopType sType) {

    // Select the stop type.
    switch (sType) {
        // To brake, write all high.
        case BRAKE: 
            isSwitching = true;
            ledcWrite(posTerm, LED_C_HIGH);
            ledcWrite(negTerm, LED_C_HIGH);
            break;
        
        // To coast write all low.
        case COAST:
            ledcWrite(posTerm, LED_C_LOW);
            ledcWrite(negTerm, LED_C_LOW);
            break;
    }
}

bool Motor::getEnableStatus() { return enabled; }

ulong Motor::getOnTime() { return onTime; }