#include "PeripheralManager.h"
#include "Device.h"

// Define task handles.
TaskHandle_t trigger_USS_task_handle = NULL;  
TaskHandle_t poll_obs_detection_uss_handle = NULL;              

/**
 * This task reads and provides the distance values measured from the ultrasonic sensors.
 * Order of reading: Front->Back->Left->Right. This order minimizes potential echoes and interference between 
 * sensors. Based on the way the sensors will be pulsed, the task will wait to be notified in the same order
 * from the interrupts: "on_xxxxx_us_echo_changed()" [xxxxx = front, back, etc.] and will act on the notifcation 
 * value that corresponds to the interrupt that just notified it. These values are defined in the header for this file.
 * @param *pvPeripheralManager a pointer to the Sensor Manager instance running from which the sensors will be pulsed.
 * each sensor will be pulsed using the HCSR04 "pulseTrigger" function. 
 */
void trigger_USS_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);

    // Grab distance sensing transducer.
    HCSR04 *transducer = manager->fetchUS(SensorID::transducer);
    bool readingGood;

    // Begin task loop.
    for(;;) {

        // Wait for notifcation before trigger.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        
    }
}

void poll_obs_detection_uss_task(void *pvPeripheralManager) {
    // Initialize task.

    // Begin task loop.
    for(;;) {

    }
}

void on_transducer_us_echo_changed(void *arg) {
    ulong currTime = micros();
    HCSR04 *singleTransducer = static_cast<HCSR04 *>(arg);
    int pinState = digitalRead(singleTransducer->getEchoPinNumber());
    if(pinState == HIGH) singleTransducer->setISRStartPulse(currTime);
    else {
        singleTransducer->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(trigger_USS_task_handle, T_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    }
}

void on_left_us_echo_changed(void *arg) {
    ulong currTime = micros();
    HCSR04 *leftSensor = static_cast<HCSR04 *>(arg);
    int pinState = digitalRead(leftSensor->getEchoPinNumber());
    if(pinState == HIGH) leftSensor->setISRStartPulse(currTime);
    else {
        leftSensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(poll_obs_detection_uss_handle, L_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    } 
} 

void on_right_us_echo_changed(void *arg) {
    ulong currTime = micros();
    HCSR04 *rightSensor = static_cast<HCSR04 *>(arg);
    int pinState = digitalRead(rightSensor->getEchoPinNumber());
    if(pinState == HIGH) rightSensor->setISRStartPulse(currTime);
    else {
        rightSensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(poll_obs_detection_uss_handle, R_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    }
}

/**
 * Create Peripheral Manager.
 * @param dev Pointer to the device who's peripherals require management.
 */
PeripheralManager::PeripheralManager(Device *dev) : dev(dev) { 
    if(dev->isTransmitter()) {
        constructBeltPeripherals();
        attachBeltInterrupts();
    }
    else {
        constructBotPeripherals();
        attachBotInterrupts();
    }
}

/**
 * Initialize the 4 US sensors, Mic, and BME.
 */
void PeripheralManager::initPeripherals(){
    initUS();
    initDriveSystem();
}

/**)
 * Initialze the Sentry's 4 Ultrasonic Sensors.
 */
void PeripheralManager::initUS() {

    // Initialize the ultrasonic sensors.
    transducerUS->init();
    leftUS->init();
    rightUS->init();
}

void PeripheralManager::attachBeltInterrupts() {
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            attachInterruptArg(
                (int) BeltPin::single_uss_echo, 
                on_transducer_us_echo_changed, 
                transducerUS, 
                CHANGE
            );
            break;

        case SocConfig::ESP32_S3_8MB :
            attachInterruptArg(
                (int) S3BeltPin::single_uss_echo, 
                on_transducer_us_echo_changed, 
                transducerUS, 
                CHANGE
            );
            break;
        
        case SocConfig::NONE :
            Serial.println("Soc Config unset. Unable to attach interrupts. Amerliorate.");
            break;

        default:
            Serial.println("Soc Config Fell through. No interrutps. Big Issue.");
            break;
    }
}

void PeripheralManager::attachBotInterrupts() {
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            attachInterruptArg(
                (int) BotPin::single_uss_echo, 
                on_transducer_us_echo_changed, 
                transducerUS, 
                CHANGE
            );

            attachInterruptArg(
                (int) BotPin::left_hcsr04_echo, 
                on_left_us_echo_changed, 
                leftUS, 
                CHANGE
            );

            attachInterruptArg(
                (int) BotPin::right_hcsr04_echo, 
                on_right_us_echo_changed, 
                rightUS, 
                CHANGE
            );
            
            break;

        case SocConfig::ESP32_S3_8MB :
            attachInterruptArg(
                (int) S3BotPin::single_uss_echo, 
                on_transducer_us_echo_changed, 
                transducerUS, 
                CHANGE
            );

            attachInterruptArg(
                (int) S3BotPin::left_hcsr04_echo, 
                on_left_us_echo_changed, 
                leftUS, 
                CHANGE
            );

            attachInterruptArg(
                (int) S3BotPin::right_hcsr04_echo, 
                on_right_us_echo_changed, 
                rightUS, 
                CHANGE
            );
            break;
        
        case SocConfig::NONE :
            Serial.println("Soc Config unset. Unable to attach interrupts. Amerliorate.");
            break;

        default:
            Serial.println("Soc Config Fell through. No interrutps. Big Issue.");
            break;
    }
}

// Per name.
void PeripheralManager::beginTasks() {

    // Create and begin all sensor based tasks.
    BaseType_t taskCreated;

    taskCreated = beginTriggerDistanceUssTask();
    if(taskCreated != pdPASS) Serial.printf("Transducer trigger task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Transducer trigger task created.");

    taskCreated = beginPollObstacleDetectionUssTask();
    if(taskCreated != pdPASS) Serial.printf("Read Ultrasonic Sensor task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Read Ultrasonic Sensor task created.");
    
}

HCSR04* PeripheralManager::fetchUS(SensorID id) {
    HCSR04 *res = NULL;
    switch (id) {
        case (SensorID::transducer) : 
            res = transducerUS;
            break;
        case (SensorID::left) : 
            res = leftUS;
            break;
        case (SensorID::right) : 
            res = rightUS;
            break;
    }
    return res;
}

// Create the task trigger the distance sensing ultrasonic transducer.
BaseType_t PeripheralManager::beginTriggerDistanceUssTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &trigger_USS_task,              // Pointer to task function.
        "trigger_transducer_Task",             // Task name.
        TaskStackDepth::tsd_POLL,       // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &trigger_USS_task_handle,       // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

// Create the task to poll the 2 obstacle detection ultrasonic sensors.
BaseType_t PeripheralManager::beginPollObstacleDetectionUssTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &poll_obs_detection_uss_task,       // Pointer to task function.
        "poll_obs_detect_USS_Task",         // Task name.
        TaskStackDepth::tsd_POLL,           // Size of stack allocated to the task (in bytes).
        this,                               // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,        // Task priority level.
        &poll_obs_detection_uss_handle,     // Pointer to task handle.
        1                                   // Core that the task will run on.
    );
    return res;
}

void PeripheralManager::constructBeltPeripherals() {
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            transducerUS = new HCSR04(
                (int) BeltPin::single_uss_trig, 
                (int) BeltPin::single_uss_echo, 
                SensorID::transducer, 
                OBS_LIM
            );
            break;

        case SocConfig::ESP32_S3_8MB :
            transducerUS = new HCSR04(
                (int) S3BeltPin::single_uss_trig, 
                (int) S3BeltPin::single_uss_echo, 
                SensorID::transducer, 
                OBS_LIM
            );
            break;
        
        case SocConfig::NONE :
            Serial.println("Soc Config unset. Amerliorate.");
            break;

        default:
            Serial.println("Soc Config Fell through. Big Issue.");
            break;
    }
}

void PeripheralManager::constructBotPeripherals() {
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            transducerUS = new HCSR04(
                (int) BotPin::single_uss_trig, 
                (int) BotPin::single_uss_echo, 
                SensorID::transducer, 
                OBS_LIM
            );

            leftUS = new HCSR04(
                (int) BotPin::left_hcsr04_trig, 
                (int) BotPin::left_hcsr04_echo,
                SensorID::left, 
                OBS_LIM 
            );

            rightUS = new HCSR04(
                (int) BotPin::right_hcsr04_trig, 
                (int) BotPin::right_hcsr04_echo, 
                SensorID::right, 
                OBS_LIM 
            );

            driveSystem = new BTS7960(
                (int) BotPin::left_mot_left_pwm,
                (int) BotPin::left_mot_right_pwm,
                (int) BotPin::right_mot_left_pwm,
                (int) BotPin::right_mot_right_pwm
            );
            break;

        case SocConfig::ESP32_S3_8MB :
            transducerUS = new HCSR04(
                (int) S3BotPin::single_uss_trig, 
                (int) S3BotPin::single_uss_echo, 
                SensorID::transducer, 
                OBS_LIM
            );

            leftUS = new HCSR04(
                (int) S3BotPin::left_hcsr04_trig, 
                (int) S3BotPin::left_hcsr04_echo,
                SensorID::left, 
                OBS_LIM 
            );

            rightUS = new HCSR04(
                (int) S3BotPin::right_hcsr04_trig, 
                (int) S3BotPin::right_hcsr04_echo, 
                SensorID::right, 
                OBS_LIM 
            );

            driveSystem = new BTS7960(
                (int) S3BotPin::left_mot_left_pwm,
                (int) S3BotPin::left_mot_right_pwm,
                (int) S3BotPin::right_mot_left_pwm,
                (int) S3BotPin::right_mot_right_pwm
            );
            break;
        
        case SocConfig::NONE :
            Serial.println("Soc Config unset. Amerliorate.");
            break;

        default:
            Serial.println("Soc Config Fell through. Big Issue.");
            break;
    }
}

void PeripheralManager::initDriveSystem() {
    driveSystem->init();
}

BaseType_t PeripheralManager::beginDriveTask() {
    BaseType_t res;
    return res;
}