// Include gaurd.
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

// Grab required headers.

#include "../HCSR04/HCSR04.h"
#include "../BTS7960/BTS7960.h"
#include "config.h"
#include <Preferences.h>

// Forward definitions.
#pragma once
class Device;

// Some quality of life type definitions.
typedef uint32_t milliSeconds;      // Milliseconds.

// Internal Task info. 
#define TTR_US 40  // Time-to-read a single ultrasonic sensor (in milliseconds).
#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US))     // The maximum time it takes to read an ultrasonic sensor (in ticks).
#define MAX_US_POLL_TIME ((4 * US_READ_TIME) + 10)              // The delay between polling all 4 ultrasonic sensors w/ some buffer time.

void IRAM_ATTR on_transducer_us_echo_changed(void *arg);        // ISR that deals with timing of front ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.
void IRAM_ATTR on_hcsr04_us_echo_changed(void *arg);              // ISR that deals with timing of left ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.

extern TaskHandle_t trig_tx_transducer_task_handle;             // Handle to task that triggers the transmitters distance measuring transducer.
extern TaskHandle_t trig_left_rx_transducer_task_handle;        // Handle to task that triggers the receivers left distance measuring transducer.
extern TaskHandle_t trig_right_rx_transducer_task_handle;       // Handle to task that triggers the receivers left distance measuring transducer.
extern TaskHandle_t poll_obs_detection_uss_handle;              // Handle to task that triggers reading the obstacle detection uss.

void trig_tx_transducer_task(void *pvPeripheralManager);        // Task function that triggers the transmitters distance measuring transducer.
void trig_left_rx_transducer_task(void *pvPeripheralManager);   // Task function that triggers the receivers left distance measuring transducer.
void trig_right_rx_transducer_task(void *pvPeripheralManager);  // Task function that triggers the receivers left distance measuring transducer.
void poll_obs_detection_uss_task(void *pvPeripheralManager);    // Task function that triggers reading the obstacle detection uss. 

/**
 * Class used to manage device peripherals.
 */
class PeripheralManager {

    //*****************************  General Management  *********************************/
    private:
        Device *dev;
        void constructBeltPeripherals();
        void constructBotPeripherals();
        void attachBeltInterrupts();
        void attachBotInterrupts();

    public:
        /**
         * Create Peripheral Manager.
         * @param dev Pointer to the device who's peripherals require management.
         */
        PeripheralManager(Device *dev);

        void initPeripherals();     // Initialize all peripherals.
        void attachInterrupts();    // Attach all interrupts.     
        void beginTasks();          // Begin all tasks.
        bool isTransmitter();       

    //************************************************************************************/
    
    //*****************************  Ultrasonic Sensors  *********************************/
    private:
        HCSR04 *txTransducer = NULL;        // Transmitting US (if device == Belt)
        HCSR04 *rightRxTransducer = NULL;   // Right receiving uss (if device == Bot).
        HCSR04 *leftRxTransducer = NULL;    // Left receiving uss (if device == Bot).
        HCSR04 *leftObsDetUS = NULL;        // Left obstacle detection uss (if device == Bot).
        HCSR04 *rightObsDetUS = NULL;       // Right obstacle detection uss (if device == Bot).

        float isrPulseDuration = -1;        // Stores the duration of the pulse captured by ISR.
        unsigned long isrPulseStart = -1;   // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = -1;     // Stores the time at which the sensor's echo has finished from ISR.    
        
        BaseType_t beginTriggerTxTransducerTask();
        BaseType_t beginTriggerLeftRxTransducerTask();
        BaseType_t beginTriggerRightRxTransducerTask();

    public:
        void initUS();
        BaseType_t beginTransducerTriggerTasks();
        BaseType_t beginPollObstacleDetectionUssTask();
        HCSR04 *fetchUS(SensorID id);
    //************************************************************************************/

    //*****************************  Drive System  *********************************/
    private:  
        BTS7960 *driveSystem;

    public:
        void initDriveSystem();
        BaseType_t beginDriveTask();
    //************************************************************************************/

};


// End include gaurd.
#endif /* PeripheralManager.h */