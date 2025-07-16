#include "HCSR04.h"

void on_echo_changed(void *arg) {
    ulong currTime = micros();
    HCSR04 *transducer = static_cast<HCSR04 *>(arg);

    // Grab the pin state.
    int pinState = digitalRead(transducer->getEchoPinNumber());

    // Track changes and act accordingly.
    if(pinState == HIGH) transducer->setISRStartPulse(currTime);
    else {
        transducer->setISREndPulse(currTime);

        // Notify the specified task.
        TaskHandle_t handle = transducer->getTaskHandle();
        NotificationMask notifValue = transducer->getReadingValidBits();
        if(handle != NULL && notifValue != UNSET) {
            BaseType_t higherPriorityWasAwoken = pdFALSE;
            xTaskNotifyFromISR(handle, notifValue, eSetBits, &higherPriorityWasAwoken);
            portYIELD_FROM_ISR(higherPriorityWasAwoken);
        }
        else {
            if(handle == NULL) log_e("HC-SR04[%d]: Null Task Handle.", transducer->identify());
            if(notifValue == UNSET) log_e("HC-SR04[%d]: Notif Value Unset.", transducer->identify());
        }
    }
}

/**
 * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
 */
void HCSR04::init() {
    // Define pin connections.
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);

    // Attach interrupt.
    attachInterruptArg(
        this->echo, 
        on_echo_changed, 
        this, 
        CHANGE
    );

    // Enable sensor for use.
    enable();
}

/**
 * Poll the sensor and store the data.
 * @return True if the reading was successful, false otherwise.
 */
bool HCSR04::readSensor(TickType_t xMaxBlockTime) {
    
    // Ensure sensor is active and task is set before reading.
    bool res = false;
    if(!active) {
        log_e("Invalid Read. Sensor(%d) inactive.", id);
        return res;
    }
    if(*taskHandlePtr == NULL) {
        log_e("Unable to read: %s. Null Task Handle.", this->getName().c_str());
        return res;
    }

    // Reset Echo and Pulse trigger for 10 us.
    //resetEchoTimestamps();
    pulseTrigger();
    
    // Wait for pulse to complete.
    ulong pulseFinishedEvent, echoHighTime;
    BaseType_t waitSuccess;
    waitSuccess = xTaskNotifyWait(this->notifValid, this->notifValid, &pulseFinishedEvent, xMaxBlockTime);
    echoHighTime = isrPulseEnd - isrPulseStart;

    // Reset values if out of range.
    if(echoHighTime >= TTR_US*1000 || echoHighTime == 0) resetEchoTimestamps();

    // Compute distance if in range.
    else if((pulseFinishedEvent & this->notifValid) == this->notifValid && waitSuccess == pdTRUE) {
        // Compute distance just measured.
        float inches = computeInches();

        // Store the last average for later comparisons.
        lastBufferAverage = averageBuffer();

        // Update the buffer.
        if(distIndex == bufferSize) distIndex = 0;
        pastDistances[distIndex++] = inches;
        res = true;
    }

    // Return.
    return res;
}

/**
 * Mark a sensor as relevant for output collection.
 */
void HCSR04::enable() {
    active = true;
}

/**
 * Mark a sensor as irrelevant for output collection.
 */
void HCSR04::disable() {
    active = false;
}

/**
 * Signal that this ultrasonic sensor has passed one or both of its 2 thresholds.
 * @return A byte where the least two significant bits represent detection threshold
 * breaches and the next 3 represent the strength of the breach.
 * Bit 0: Obstacle detection,
 * Bit 1: Human presence estimation,
 * Bit 2: Strong breach (certain that the barrier has been broken).
 * Bit 3: Moderate breach.
 * Bit 4: Weak breach.
 */
char HCSR04::passedThreshold() {
    char flag = 0x00;

    // Only check thresholds if sensor is active.
    if(this->active == false) return flag;

    // Check obstacle detection threshold. (This is checked against most recent distance instead of the buffers).
    if(getDistanceReading() <= obstacleDetectionThreshold) flag |= OBSTACLE_THRESHOLD_BREACHED;

    // Check human presence estimation threshold.
    float currBufferAvg = averageBuffer();
    float HpeCheck = presenceDetectionThreshold/currBufferAvg - 1;
    if(HpeCheck >= HPE_PERCENT_DIFF/100.0) {
        // Set the bit indicating human presence was detected.
        flag |= PRESENCE_THRESHOLD_BREACHED;

        // Check the difference between current and last buffer averages to determine the strength/confidence of presence.
        float bufferPercentDiff = abs(currBufferAvg - lastBufferAverage)/lastBufferAverage;

        // Greater than a 10% difference between buffers while presence is detected strongly indicates presence (and motion within boundary).
        if(bufferPercentDiff > HPE_STRONG_PERCENT/100.0) {
            flag |= STRONG_PRESENCE_BREACH;
            //Serial.printf("😁diff: %f->Strong Presence Detected->Flag = 0x%x\n", bufferPercentDiff, flag);
        }

        // Less than a 5% difference between buffers while presence is detected weakly indicates presence (and motion within boundary).
        else if(bufferPercentDiff < HPE_WEAK_PERCENT/100.0) {
            flag |= WEAK_PRESENCE_BREACH;
            //Serial.printf("😁diff: %f->Weak Presence Detected->Flag = 0x%x\n", bufferPercentDiff, flag);
        }

        // In between a 5-10% difference between buffers while presence is detected moderately indicates presence (and motion within boundary).
        else {
            flag |= MODERATE_PRESENCE_BREACH;
            //Serial.printf("😁diff: %f->Moderate Presence Detected->Flag = 0x%x\n", bufferPercentDiff, flag);
        }
        
    }

    // Return.
    return flag;
}

/**
 * Take the avarage of this sensors past distances buffer.
 * @return The average value of this sensors past distances.
 */
float HCSR04::averageBuffer() {
    float sum = 0;
    for(int i = 0; i < bufferSize; i++) sum += pastDistances[i];
    return sum/bufferSize;
}

/**
 * Check if this sensor is active or not.
 * @return True if sensor is active, false otherwise.
 */
bool HCSR04::isActive() {
    return active;
}

/**
 * Set this sensors obstacle detection threshold.
 * @param threshold The new threshold to be integrated.
 */
void HCSR04::setObstacleDetectionThreshold(float threshold) {
    obstacleDetectionThreshold = threshold;
}

/**
 * Retrieve this sensors obstacle detection threshold.
 */
float HCSR04::getObstacleDetectionThreshold() {
    return obstacleDetectionThreshold;
}

/**
 * Retrieve this sensors human presence threshold.
 */
float HCSR04::getHpeThreshold() { return presenceDetectionThreshold; }

/**
 * Pulse this ultrasonic sensors trigger pin to initiate measurements.
 */
void HCSR04::pulseTrigger() {
    // Pulse trigger for 10 us.
    digitalWrite(trigger, LOW);
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
}

float HCSR04::computeInches() {
    float isrPulseDuration = (isrPulseEnd - isrPulseStart) * 1.0;
    float distanceInInches = (isrPulseDuration/2) / 74;
    bool isTransducer = (this->id == SensorID::leftRxTransducer || this->id == SensorID::rightRxTransducer);
    return (isTransducer) ? distanceInInches*2 : distanceInInches;
}

void HCSR04::resetEchoTimestamps() {
    isrPulseEnd = 1;
    isrPulseStart = 1; 
}

void HCSR04::setISRStartPulse(ulong start) {
    isrPulseStart = start;
}

void HCSR04::setISREndPulse(ulong end) {
    isrPulseEnd = end;
}

float HCSR04::getDistanceReading() { 
    float res = -1;
    if(!active) return res;
    if(distIndex > 0) res = pastDistances[distIndex - 1]; 
    else res = pastDistances[bufferSize - 1]; // Index should get the last element in the buffer.
    return res;
}

float HCSR04::getLastBufferAverage() { return lastBufferAverage; }

int HCSR04::getTriggerPinNumber() { return trigger; }
int HCSR04::getEchoPinNumber() { return echo; }

bool HCSR04::isTransducer() { return (id != SensorID::leftObsDet) && (id != SensorID::rightObsDet); }

SensorID HCSR04::identify() { return id; }

String HCSR04::getName() {
    String res;
    switch (id) {
        case (SensorID::txTransducer):
            res = "tx_td";
            break;

        case (SensorID::leftRxTransducer):
            res = "l_rx_td";
            break;

        case (SensorID::rightRxTransducer):
            res = "r_rx_td";
            break;

        case (SensorID::leftObsDet) : 
            res = "l_od_us";
            break;

        case (SensorID::rightObsDet) : 
            res = "r_od_us";
            break;
    }
    return res;
}

void HCSR04::attachTaskHandle(TaskHandle_t *handlePtr) { this->taskHandlePtr = handlePtr; }

TaskHandle_t HCSR04::getTaskHandle() { return *(this->taskHandlePtr); }

NotificationMask HCSR04::getReadingValidBits() { return this->notifValid; }

NotificationMask HCSR04::getReadingInvalidBits() { return this->notifInvalid; }

ulong HCSR04::getISRStartPulse() {return this->isrPulseStart; }

ulong HCSR04::getISREndPulse() { return this->isrPulseEnd; }
