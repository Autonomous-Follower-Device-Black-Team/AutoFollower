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
                    Utility Functions.
**********************************************************/
bool initMutex(SemaphoreHandle_t *mutexPtr);
BaseType_t grabMutex(SemaphoreHandle_t *mutexPtr, TickType_t timeout = portMAX_DELAY);
BaseType_t releaseMutex(SemaphoreHandle_t *mutexPtr);

void dump_rx_diff_info(signed long long lst, signed long long rst, signed long long let, signed long long ret);

/*********************************************************
        Ultrasonic Sensor Subsystem Task Info.
**********************************************************/
#define RX_TTR_OFFSET 5     // Offset from normal ultrasonic readtime for receiver only.    
#define RX_US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US - RX_TTR_OFFSET))      // The maximum time it takes to read a receiving only transducer (in ticks).
#define MAX_US_POLL_TIME ((4 * US_READ_TIME) + 10)                                  // The delay between polling all 4 ultrasonic sensors w/ some buffer time.

#define RX_DIFF_INVALID 300
#define BUF_INV ((float) -710.0)

extern EventGroupHandle_t rx_trig_sync_group;                   // Handle to the event group that syncs the triggers of the two receivers.  
extern EventGroupHandle_t rx_echo_time_group;                   // Handle to the event group that tracks the timing of rx echoes.
extern SemaphoreHandle_t rx_echo_time_mutex;                    // Handle to the Mutex that controls access to USS time group.
extern SemaphoreHandle_t echo_diff_buffer_mutex;                // Handle to the Mutex that controls access to echo difference buffer.
extern SemaphoreHandle_t drive_system_mutex;                    // Handle to the Mutex that controls access to the drive system.

extern TaskHandle_t trig_tx_transducer_task_handle;             // Handle to task that triggers the transmitters distance measuring transducer.
extern TaskHandle_t trig_left_rx_transducer_task_handle;        // Handle to task that triggers the receivers left distance measuring transducer.
extern TaskHandle_t trig_right_rx_transducer_task_handle;       // Handle to task that triggers the receivers right distance measuring transducer.
extern TaskHandle_t poll_obs_detection_uss_handle;              // Handle to task that triggers reading the obstacle detection uss.
extern TaskHandle_t left_right_rx_diff_task_handle;             // Handle to task that computes difference between echo signal receive times and notifies PID.

void trig_tx_transducer_task(void *pvPeripheralManager);        // Task function that triggers the transmitters distance measuring transducer.
void trig_rx_transducer_task(void *pvPeripheralManager);        // Task function that triggers the receiver distance measuring transducers (left/right).
void poll_obs_detection_uss_task(void *pvPeripheralManager);    // Task function that triggers reading the obstacle detection uss. 
void left_right_rx_diff_task(void *pvPeripheralManager);        // Task function that deals w/ the difference in echo receives.     

struct _rx_echo_dist_history {
    bool readyForUse;
    uint8_t size;
    uint8_t index;
    signed long long *echoBuffer;
    float *distBuffer; 
};
typedef struct _rx_echo_dist_history rxHistoryBuffer;

struct _buffer_averages {
    float echoDiffAvg;
    float distAvg;
};
typedef struct _buffer_averages BufferAverages;

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
#define MIN_SPEED 80            // 100
#define MAX_SPEED 200           // 400
#define DEFAULT_SPEED 100       // 150
#define TARGET_DIST_IN 3 * 12
#define MAX__FOLLOW_DIST_IN 9 * 12
#define ECHO_DIFF_LOWER_BOUND -75
#define ECHO_DIFF_UPPER_BOUND 75
#define DEFAULT_KP 0.1
#define DEFAULT_KZ 50

extern TaskHandle_t obs_det_stop_task_handle;
extern TaskHandle_t mvmt_manager_task_handle;

void obs_det_stop_task(void *pvPeripheralManager);       // Task fucntion to stop motors when obstacles are detected.
void mvmt_manager_task(void *pvPeripheralManager);       // Task function to control motor motion.

struct _bang_bang_cfg {

    uint16_t minSpeed;      // Minimum allowable speed for each robot wheel
    uint16_t maxSpeed;      // Maximum allowable speed for each robot wheel.
    uint16_t defSpeed;      // Default speed for each robot wheel.
    float targetDist;       // Average target distance (in inches).
    float maxDist;          // Maximum distance allowable for valid movement (in inches).

    /**
     * Lower Bound of the echo difference to be used in comparison and classification
     * of which of the two receivers got the ping from the transmitter first.
     */
    uint8_t edLower;

    /**
     * Upper Bound of the echo difference to be used in comparison and classification
     * of which of the two receivers got the ping from the transmitter first.
     */
    uint8_t edUpper;
    
    /**
     * The turning (or differential) multiplier. Multiplies the Echo Diff to get a proportional steering
     * term to be used in calculating indidividual wheel speeds.
     */
    float kp;     

    /**
     * The distance measured multiplier. Multiplies the average distance measured to get
     * a term that increases speed depending on how far away the receiver is.
     */
    float kz;       
};
typedef struct _bang_bang_cfg BangBangCtrlConfig;

/**
 * Class used to manage device peripherals.
 */
class PeripheralManager {

    //*****************************  General Management  *********************************/
    private:
        Device *dev;
        void constructBeltPeripherals();
        void constructBotPeripherals();

    public:
        /**
         * Create Peripheral Manager.
         * @param dev Pointer to the device who's peripherals require management.
         */
        PeripheralManager(Device *dev);

        void initPeripherals();     // Initialize all peripherals.
        void beginTasks();          // Begin all tasks.
        void createSemaphores();    // Create all semaphores.
        void createEventGroups();   // Create all event groups.
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
        rxHistoryBuffer rxHistoryBuffers;

        void initRxHistoryBuffers(int size = 10);

        BaseType_t beginTriggerTxTransducerTask();
        BaseType_t beginTriggerLeftRxTransducerTask();
        BaseType_t beginTriggerRightRxTransducerTask();
        BaseType_t beginTransducerTriggerTasks();
        BaseType_t beginPollObstacleDetectionUssTask();
        BaseType_t beginReceiverEchoDiffTask();

    public:
        void initUS();
        HCSR04 *fetchUS(SensorID id);
        HCSR04 *fetchTransducer(TaskHandle_t handle);
        
        void fillUsTimingGroup(SensorID id);
        USTimeGroup *getUsTimingGroup();
        void addToBuffers(signed long long echoDiff, float avgDist);
        BufferAverages getDiffBufferAverages();
        bool isBufferReadyForUse();

    //************************************************************************************/

    //*****************************  Drive System  *********************************/
    private:  
        BTS7960 *driveSystem;
        BangBangCtrlConfig followingLogicConfig;

        BaseType_t beginEmergencyStopTask();
        BaseType_t beginMovementManagerTask();

        void initBangBangCtrlConfig();

    public:
        void initDriveSystem();
        BTS7960 *getDriveSystem();

        void setFollowingLogicDifferentialGain(uint8_t k);
        void setFollowingLogicDistanceGain(uint8_t k);
        BangBangCtrlConfig *getFollowingLogicConfig(); 

    //************************************************************************************/

};


// End include gaurd.
#endif /* PeripheralManager.h */