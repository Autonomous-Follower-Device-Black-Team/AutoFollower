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
typedef uint32_t NotificationMask;  // Mask to delineate between Notifcations.
typedef uint32_t milliSeconds;      // Milliseconds.

// Internal Task info. 

// USS Identification.
enum _sensor_id : uint8_t {
    transducer,     // Id for transmitter or receiver.
    left,           // Id for left USS.
    right           // Id for right USS
};
typedef enum _sensor_id SensorID;

#define T_US_READY ((NotificationMask) 0x0001)  // Transducer ultrasonic sensor notification.
#define L_US_READY ((NotificationMask) 0x0100)  // Left ultrasonic sensor notification.
#define R_US_READY ((NotificationMask) 0x1000)  // Right ultrasonic sensor notification.

#define TTR_US 40  // Time-to-read a single ultrasonic sensor (in milliseconds).
#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US))     // The maximum time it takes to read an ultrasonic sensor (in ticks).
#define MAX_US_POLL_TIME ((4 * US_READ_TIME) + 10)              // The delay between polling all 4 ultrasonic sensors w/ some buffer time.

void IRAM_ATTR on_transducer_us_echo_changed(void *arg);     // ISR that deals with timing of front ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.
void IRAM_ATTR on_left_us_echo_changed(void *arg);      // ISR that deals with timing of left ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.
void IRAM_ATTR on_right_us_echo_changed(void *arg);     // ISR that deals with timing of right ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.

extern TaskHandle_t trigger_USS_task_handle;                // Handle to task that triggers the distance measuring transducer.
void trigger_USS_task(void *pvPeripheralManager);               // Task function that triggers the distance measuring transducer.

extern TaskHandle_t poll_obs_detection_uss_handle;          // Handle to task that triggers reading the obstacle detection uss.
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

    //************************************************************************************/
    
    //*****************************  Ultrasonic Sensors  *********************************/
    private:
        HCSR04 *transducerUS = NULL;   // Device transmitter or receiver.
        HCSR04 *leftUS = NULL;         // Left uss (if device == Belt).
        HCSR04 *rightUS = NULL;        // Right uss (if device == Belt).

        float isrPulseDuration = -1;      // Stores the duration of the pulse captured by ISR.
        unsigned long isrPulseStart = -1; // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = -1;   // Stores the time at which the sensor's echo has finished from ISR.    

    public:
        void initUS();
        BaseType_t beginTriggerDistanceUssTask();
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