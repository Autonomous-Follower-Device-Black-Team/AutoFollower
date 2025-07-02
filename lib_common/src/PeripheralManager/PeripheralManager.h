// Include gaurd.
#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

// Grab required headers.

#include "../HCSR04/HCSR04.h"
#include "../BTS7960/BTS7960.h"
#include "config.h"
#include <Preferences.h>

// Forward definitions.
#pragma once
class Device;

/*********************************************************
        Ultrasonic Sensor Subsystem Task Info.
**********************************************************/
#define TTR_US 40  // Time-to-read a single ultrasonic sensor (in milliseconds).
#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US))     // The maximum time it takes to read an ultrasonic sensor (in ticks).
#define MAX_US_POLL_TIME ((4 * US_READ_TIME) + 10)              // The delay between polling all 4 ultrasonic sensors w/ some buffer time.

void IRAM_ATTR on_hcsr04_us_echo_changed(void *arg);            // ISR that deals with timing of left ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.

extern EventGroupHandle_t rx_echo_time_group;                   // Handle to the event group that tracks the timing of rx echoes.
extern SemaphoreHandle_t rx_echo_time_mutex;                // Handle to the semaphore that controls access to USS time group.

extern TaskHandle_t trig_tx_transducer_task_handle;             // Handle to task that triggers the transmitters distance measuring transducer.
extern TaskHandle_t trig_left_rx_transducer_task_handle;        // Handle to task that triggers the receivers left distance measuring transducer.
extern TaskHandle_t trig_right_rx_transducer_task_handle;       // Handle to task that triggers the receivers left distance measuring transducer.
extern TaskHandle_t poll_obs_detection_uss_handle;              // Handle to task that triggers reading the obstacle detection uss.
extern TaskHandle_t left_right_rx_diff_task_handle;             // Handle to task that computes difference between echo signal receive times and notifies PID.

void trig_tx_transducer_task(void *pvPeripheralManager);        // Task function that triggers the transmitters distance measuring transducer.
void trig_left_rx_transducer_task(void *pvPeripheralManager);   // Task function that triggers the receivers left distance measuring transducer.
void trig_right_rx_transducer_task(void *pvPeripheralManager);  // Task function that triggers the receivers left distance measuring transducer.
void poll_obs_detection_uss_task(void *pvPeripheralManager);    // Task function that triggers reading the obstacle detection uss. 
void left_right_rx_diff_task(void *pvPeripheralManager);        // Task function that deals w/ the difference in echo receives.     

struct _us_times {
    signed long long leftStartTime;    // Left Rx Transducer Echo End Time.
    signed long long rightStartTime;   // Right Rx Transducer Echo End Time.
    signed long long leftEndTime;      // Left Rx Transducer Echo End Time.
    signed long long rightEndTime;     // Right Rx Transducer Echo End Time.
};
typedef struct _us_times USTimeGroup;

/*********************************************************
            Drive Subsystem Task Info.
**********************************************************/
extern TaskHandle_t obs_det_stop_task_handle;
extern TaskHandle_t mvmt_manager_task_handle;

void obs_det_stop_task(void *pvPeripheralManager);       // Task fucntion to stop motors when obstacles are detected.
void mvmt_manager_task(void *pvPeripheralManager);       // Task function to control motor motion.

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
        USTimeGroup usTimingGroup;
        
        BaseType_t beginTriggerTxTransducerTask();
        BaseType_t beginTriggerLeftRxTransducerTask();
        BaseType_t beginTriggerRightRxTransducerTask();
        BaseType_t beginTransducerTriggerTasks();
        BaseType_t beginPollObstacleDetectionUssTask();
        BaseType_t beginReceiverEchoDiffTask();

    public:
        void initUS();
        HCSR04 *fetchUS(SensorID id);
        void fillUsTimingGroup(SensorID id);
        USTimeGroup *getUsTimingGroup();

    //************************************************************************************/

    //*****************************  Drive System  *********************************/
    private:  
        BTS7960 *driveSystem;

        BaseType_t beginEmergencyStopTask();
        BaseType_t beginMovementManagerTask();

    public:
        void initDriveSystem();
        BTS7960 *getDriveSystem();
    //************************************************************************************/

};


// End include gaurd.
#endif /* PeripheralManager.h */