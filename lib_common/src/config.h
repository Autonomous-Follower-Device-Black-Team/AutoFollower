#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define BAUD_RATE 115200

typedef uint32_t milliSeconds;
#define TTR_US 40  // Time-to-read a single ultrasonic sensor (in milliseconds).
#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US))     // The maximum time it takes to read an ultrasonic sensor (in ticks).

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
    single_uss_trig = 4,
    single_uss_echo = 5,
    left_hcsr04_trig = 6,
    left_hcsr04_echo = 7,
    right_hcsr04_trig = 15,
    right_hcsr04_echo = 16,
    left_mot_left_pwm = 17,
    left_mot_right_pwm = 18,
    right_mot_left_pwm = 40,
    right_mot_right_pwm = 41
};
typedef _bot_pins_s3 S3BotPin;

/**
 * Bot (Receiver) Pin defintions - ESP32 Config.
 */
enum class _bot_pins : uint8_t {
    single_uss_trig = 36,
    single_uss_echo = 39,
    left_hcsr04_trig = 34,
    left_hcsr04_echo = 35,
    right_hcsr04_trig = 32,
    right_hcsr04_echo = 33,
    left_mot_left_pwm = 25,
    left_mot_right_pwm = 26,
    right_mot_left_pwm = 17,
    right_mot_right_pwm = 16,
    rgbLed = 38
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
    single_uss_trig = 36,
    single_uss_echo = 39,
};
typedef _belt_pins BeltPin;


// Size of the stack allocated on the heap to a task (in bytes).
enum TaskStackDepth {
    tsd_MAX = 16384,        // Maximum size given to a task.
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