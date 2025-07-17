#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define af_SSID "SEEMS"
#define af_PASSWORD "@Ucf2025"

#define TESTING_LEFT_RX_ONLY 0
#define TESTING_RIGHT_RX_ONLY 0

#define RX_DRIVE_SYSTEM_ON 1
#define RX_ULTRASONIC_SYSTEM_ON 1
#define TX_ULTRASONIC_SYSTEM_ON 1
#define RX_DEBUG 0
#define RX_DRIVE_DEBUG 0
#define DUMP_RX_DIFF 0

#define BAUD_RATE 115200

typedef uint32_t NotificationMask;  // Mask to delineate between Notifcations.
typedef uint32_t milliSeconds;

#define UNSET ((NotificationMask) 0xFFFF)
#define T_US_READY ((NotificationMask) (1 << 0))        // Transducer ultrasonic sensor notification.
#define L_US_READY ((NotificationMask) (1 << 1))        // Left ultrasonic sensor notification.
#define R_US_READY ((NotificationMask) (1 << 2))        // Right ultrasonic sensor notification.

#define L_TD_VALID ((NotificationMask) (1 << 3))        // Left receiving transducer is valid notification.
#define R_TD_VALID ((NotificationMask) (1 << 4))        // Right receiving transducer notification.
#define L_TD_READY ((NotificationMask) 1 << 7)          // Left receiving transducer data ready.
#define R_TD_READY ((NotificationMask) 1 << 8)          // Right Receiving transducer data ready.

#define MOT_E_STOP ((NotificationMask) (1 << 9))         // Emergency stop notification.
#define MOT_RESUME ((NotificationMask) (1 << 10))        // Resume movemment notification.
#define TRIG_L_RX ((NotificationMask) (1 << 11))         // Trigger left receiving transducer notification.
#define TRIG_R_RX ((NotificationMask) (1 << 12))         // Trigger right receiving transducer notification.
#define ECHO_DIFF_READY ((NotificationMask) 1 << 13)     // Rx echo duration difference ready to be processed.

#define TD_READY (L_TD_READY | R_TD_READY)               // Both Rx transducers ready to be used.
#define TD_VALID (L_TD_VALID | R_TD_VALID)
#define TD_INVALID (!L_TD_VALID | !R_TD_VALID)

#define TRIG_RX (TRIG_L_RX | TRIG_R_RX)                 // Receiver trigger syncing notification.

//#define TTR_US 40  // Time-to-read a single ultrasonic sensor (in milliseconds).
//#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US))     // The maximum time it takes to read an ultrasonic sensor (in ticks).

/**
 * Identify which ESP32 SoC is in Use.
 */
enum class _soc_config {
    NONE,
    ESP32_4MB,
    ESP32_S3_8MB
};
typedef _soc_config SocConfig;

/**
 * Bot (Receiver) Pin defintions - ESP32-S3 Config.
 */
enum class _bot_pins_s3 : uint8_t {
    left_us_transducer_trig = 41,
    left_us_transducer_echo = 42,
    right_us_transducer_trig = 2,
    right_us_transducer_echo = 1,

    left_hcsr04_trig = 13,
    left_hcsr04_echo = 10,
    right_hcsr04_trig = 12,
    right_hcsr04_echo = 11,

    left_mot_left_pwm = 4,
    left_mot_right_pwm = 5,
    right_mot_left_pwm = 18,
    right_mot_right_pwm = 17,

    rgbLed = 38
};
typedef _bot_pins_s3 S3BotPin;

/**
 * Bot (Receiver) Pin defintions - ESP32 Config.
 */
enum class _bot_pins : uint8_t {
    left_us_transducer_trig = 36,
    left_us_transducer_echo = 39,
    right_us_transducer_trig = 27,
    right_us_transducer_echo = 14,

    left_hcsr04_trig = 34,
    left_hcsr04_echo = 35,
    right_hcsr04_trig = 32,
    right_hcsr04_echo = 33,

    left_mot_left_pwm = 25,
    left_mot_right_pwm = 26,
    right_mot_left_pwm = 17,
    right_mot_right_pwm = 16
};
typedef _bot_pins BotPin;

 /**
 * Belt (Transmitter) Pin defintions - ESP32-S3 Config.
 */
enum class _belt_pins_s3 : uint8_t {
    single_uss_trig = 4,
    single_uss_echo = 5,
    rgbLed = 38
};
typedef _belt_pins_s3 S3BeltPin;

 /**
 * Belt (Transmitter) Pin defintions - ESP32 Config.
 */
enum class _belt_pins : uint8_t {
    single_uss_trig = 33,
    single_uss_echo = 32,
};
typedef _belt_pins BeltPin;


// Size of the stack allocated on the heap to a task (in bytes).
enum TaskStackDepth {
    tsd_MAX = 16384,        // Maximum size given to a task.
    tsd_TRIG = 4096,
    tsd_POLL = 6000,        // Size given to tasks who read sensors.
    tsd_SET = 5000,         // Size given to tasks who simply set values.
    tsd_TRANSMIT = 6000,    // Size given to tasks who transmit information to firebase.
    tsd_RECEIVE = 8000,     // Size given to tasks who receive information from firebase.
    tsd_DRIVE = 10000       // Size given to tasks who drive the Sentry's Locomotion.
};

// Priority level of a task.
enum TaskPriorityLevel {
    tpl_LOW = 1,
    tpl_MEDIUM_LOW = 5,
    tpl_MEDIUM = 10,
    tpl_MEDIUM_HIGH = 15,
    tpl_HIGH = 20
};

#endif /* CONFIG_H */