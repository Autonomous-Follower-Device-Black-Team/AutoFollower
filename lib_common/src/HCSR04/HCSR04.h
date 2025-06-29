// Include guard.
#ifndef HCSR04_H
#define HCSR04_H

// Grab libraries. 
#include <Arduino.h>
#include <Preferences.h>

#define HPE_PERCENT_DIFF 2      // Meaningful percent difference between current buffer average and HPE Threshold (in %).
#define HPE_WEAK_PERCENT 5      // Percent difference between current and last buffer averages weakly indicating presence (in %).
#define HPE_STRONG_PERCENT 10   // Percent difference between current and last buffer averages strongly indicating presence (in %).
#define DEF_HP_EST_LIM 10

#define OBS_LIM 30      // USS obstacle detection limit (inches).
#define OBSTACLE_THRESHOLD_BREACHED  0x0001     // Mask representing that the Obstacle Detection Threshold of an HCSR04 Sensor has been passed.
#define PRESENCE_THRESHOLD_BREACHED  0x0002     // Mask representing that the Presence Detection Threshold of an HCSR04 Sensor has been passed.
#define STRONG_PRESENCE_BREACH 0x10             // Mask for when HPE breach is strong.
#define MODERATE_PRESENCE_BREACH 0x08           // Mask for when HPE breach is moderate.
#define WEAK_PRESENCE_BREACH 0x04               // Mask for when HPE breach is weak.

typedef uint32_t NotificationMask;  // Mask to delineate between Notifcations.
#define UNSET ((NotificationMask) 0xFFFF)
#define T_US_READY ((NotificationMask) 0x0001)  // Transducer ultrasonic sensor notification.
#define L_US_READY ((NotificationMask) 0x0100)  // Left ultrasonic sensor notification.
#define R_US_READY ((NotificationMask) 0x1000)  // Right ultrasonic sensor notification.

// USS Identification.
enum _sensor_id : uint8_t {
    txTransducer,       // Transmitter US Transducer.
    leftRxTransducer,   // Receiver Left US Transducer.
    rightRxTransducer,  // Receiver Right US Transducer.
    leftObsDet,         // Receiver Left HC-SR04 fro obstacle detection.
    rightObsDet         // Receiver Right HC-SR04 for obstacle detection.
};
typedef enum _sensor_id SensorID;

/**
 * Class representing the HC-SR04 Ultrasonic Sensors used as obstacle and presence detectors.
 */
class HCSR04 {

    private:
        /**
         * Field that serves as an identifier for this HC-SR04.
         */
        const SensorID id;
        
        /**
         * Task that this HC-SR04 will be accessed from.
         */
        TaskHandle_t taskHandle = NULL;

        /**
         * Notification value for this sensor to be used with its calling task.
         */
        NotificationMask notif = UNSET;

        /**
         * Trigger pin of this HC-SR04.
         */
        const int trigger;

        /**
         * Echo pin of this HC-SR04.
         */
        const int echo;

        /**
         * The distance from the sensor (in inches) that an obstacle must be to be "detected".
         */
        float obstacleDetectionThreshold = -1.0;

        /**
         * The distance from the sensor (in inches) that an object/person must be to be "detected".
         */
        float presenceDetectionThreshold = DEF_HP_EST_LIM;

        /**
         * Quantifies if this sensor is on or not (should be polled or not).
         */
        bool active = false;

        /**
         * Index into the distance buffer pointing to the last distance measured.
         */
        int distIndex = 0;
        
        /**
         * Last average of the buffer.
         */
        float lastBufferAverage = 0;

        /**
         * Size of distance buffer.
         */
        static constexpr int bufferSize = 5;

        /**
         * Buffer of past distances measured.
         */
        float pastDistances[bufferSize];

        unsigned long isrPulseStart = 0; // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = 0;   // Stores the time at which the sensor's echo has finished from ISR.    

        /**
         * Pulse this ultrasonic sensor's trigger pin to initiate a measurement.
         */
        void pulseTrigger();

        /**
         * Compute the distance in inches measured by the sensor.
         */
        float computeInches();

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
         * @param trigger The trigger pin of this sensor.
         * @param echo The echo pin of this sensor.
         * @param id The unique Id for this ultrasonic sensor.
         * @param obstacleDetectionThreshold The distance from the sensor (in inches) that an obstacle must be to be "detected".
         * @param notif Notification value to be used by this sensor from within its calling task.
         */
        HCSR04(int trigger, int echo, SensorID id, int obstacleDetectionThreshold, NotificationMask notif) : 
            trigger(trigger), 
            echo(echo), 
            id(id), 
            obstacleDetectionThreshold(obstacleDetectionThreshold),
            notif(notif) {};

        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        void init();

        /**
         * Poll the sensor and store the data. This must only be called from within a task.
         * @param xMaxBlockTime The maximum time allotted to read a sensor.
         * @return True if the reading was successful, false otherwise.
         */
        bool readSensor(TickType_t xMaxBlockTime);

        /**
         * Mark a sensor as relevant for output collection.
         */
        void enable();

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        void disable();

        /**
         * Retrieve this sensors Human presence estimation threshold.
         */
        float getHpeThreshold();

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
        char passedThreshold();

        /**
         * Take the avarage of this sensors past distances buffer.
         * @return The average value of this sensors past distnaces.
         */
        float averageBuffer();

        /**
         * Check if this sensor is active or not.
         * @return True if sensor is active, false otherwise.
         */
        bool isActive();

        /**
         * Set this sensors obstacle detection threshold.
         * @param threshold The new threshold to be integrated.
         */
        void setObstacleDetectionThreshold(float threshold);

        /**
         * Retrieve this sensors obstacle detection threshold.
         */
        float getObstacleDetectionThreshold();

        /**
         * For purposes of sensor reading in the appropriate ISR from the Sensor Manager.
         * Sets the start time of the echo pulse once it begins.
         * @param start This is the start time of the echo pulse once this sensor has begun measuring.
         */
        void setISRStartPulse(ulong start);

        /**
         * For purposes of sensor reading in the appropriate ISR from the Sensor Manager.
         * Sets the end time of the echo pulse once it begins.
         * @param end This is the end time of the echo pulse once this sensor has begun measuring.
         */
        void setIRSEndPulse(ulong end);

        /**
         * Get the last distance reading.
         * @return The last known distance reading from this sensor.
         */
        float getDistanceReading();

        float getLastBufferAverage();

        int getTriggerPinNumber();
        int getEchoPinNumber();

        bool isTransducer();
        SensorID identify();
        void attachTaskHandle(TaskHandle_t handle);
        TaskHandle_t getTaskHandle();
        NotificationMask getNotifValue();
};

// End include guard.
#endif /*HCSR04.h*/