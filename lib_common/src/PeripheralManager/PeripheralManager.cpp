#include "PeripheralManager.h"
#include "Device.h"

// Define task handles.
TaskHandle_t trig_tx_transducer_task_handle = NULL;  
TaskHandle_t trig_left_rx_transducer_task_handle = NULL;
TaskHandle_t trig_right_rx_transducer_task_handle = NULL;
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
void trig_tx_transducer_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    HCSR04 *transducer = manager->fetchUS(SensorID::txTransducer);
    bool readingGood;

    // Begin task loop.
    for(;;) {

        // Wait for notifcation from tirgger timer before trigger.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        // Trigger the transmitter.
        readingGood = transducer->readSensor(US_READ_TIME);

        // Log triggers and errant succesful readings if they exist. 
        // ReadingGood will always be false due to the mutilation of the sensor into tx only.
        if(!readingGood) log_e("Tx triggered Succesfully.");
        else log_e("Tx Trigger Issue.");
    }
}

void trig_left_rx_transducer_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    HCSR04 *transducer = manager->fetchUS(SensorID::leftRxTransducer); 

    bool readingGood;
    float instDistance, avgDistance;

    for(;;) {

        // Wait for notifcation from tirgger timer before trigger.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        readingGood = transducer->readSensor(US_READ_TIME);
        if(readingGood) {
            instDistance = transducer->getDistanceReading() * 2;
            avgDistance = transducer->getLastBufferAverage() * 2;
            Serial.printf("Left Rx: Distance: %f, Average: %f\n", instDistance, avgDistance);
        }
        else Serial.println("Left Rx Failed.");
    }
}

void trig_right_rx_transducer_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    HCSR04 *transducer = manager->fetchUS(SensorID::rightRxTransducer); 

    bool readingGood;
    float instDistance, avgDistance;

    for(;;) {

        // Wait for notifcation from tirgger timer before trigger.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        readingGood = transducer->readSensor(US_READ_TIME);
        if(readingGood) {
            instDistance = transducer->getDistanceReading() * 2;
            avgDistance = transducer->getLastBufferAverage() * 2;
            Serial.printf("Right Rx: Distance: %f, Average: %f\n", instDistance, avgDistance);
        }
        else Serial.println("Right Rx Failed.");
    }
}

void poll_obs_detection_uss_task(void *pvPeripheralManager) {
    // Initialize task.

    // Begin task loop.
    for(;;) {
        vTaskDelay(1000);
    }
}

void on_transducer_us_echo_changed(void *arg) {
    ulong currTime = micros();
    HCSR04 *transducer = static_cast<HCSR04 *>(arg);
    int pinState = digitalRead(transducer->getEchoPinNumber());
    if(pinState == HIGH) transducer->setISRStartPulse(currTime);
    else {
        transducer->setIRSEndPulse(currTime);
        TaskHandle_t handle = transducer->getTaskHandle();
        NotificationMask notifValue = transducer->getNotifValue();
        if(handle != NULL && notifValue != UNSET) {
            BaseType_t higherPriorityWasAwoken = pdFALSE;
            xTaskNotifyFromISR(transducer->getTaskHandle(), transducer->getNotifValue(), eSetBits, &higherPriorityWasAwoken);
            portYIELD_FROM_ISR(higherPriorityWasAwoken);
        }
        else {
            if(handle == NULL) log_e("Transducer(%d): Null Task Handle.", transducer->identify());
            if(notifValue == UNSET) log_e("Transducer(%d): Notif Value Unset.", transducer->identify());
        }
    }
}

void on_hcsr04_us_echo_changed(void *arg) {
    ulong currTime = micros();
    HCSR04 *sensor = static_cast<HCSR04 *>(arg);
    int pinState = digitalRead(sensor->getEchoPinNumber());
    if(pinState == HIGH) sensor->setISRStartPulse(currTime);
    else {
        sensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(sensor->getTaskHandle(), sensor->getNotifValue(), eSetBits, &higherPriorityWasAwoken);
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
        log_e("Belt Peripheral Setup Complete.");
    }
    else {
        constructBotPeripherals();
        log_e("Bot Peripheral Setup Complete.");
    }
}

/**
 * Initialize the US System and Drive System.
 */
void PeripheralManager::initPeripherals(){
    initUS();
    initDriveSystem();
}

void PeripheralManager::attachInterrupts() {
    if(dev->isTransmitter()) {
        attachBeltInterrupts();
        log_e("Belt interrupts attached.");
    }
    else {
        attachBotInterrupts();
        log_e("Bot interrupts attached.");
    }   
}

/**
 * Initialze the Ultrasonic Sensors.
 */
void PeripheralManager::initUS() {

    // Initialize the ultrasonic sensors.
    if(dev->isTransmitter()) {
        txTransducer->init();
        txTransducer->attachTaskHandle(trig_tx_transducer_task_handle);
    }
    else {
        leftRxTransducer->init();
        rightRxTransducer->init();
        leftObsDetUS->init();
        rightObsDetUS->init();

        leftRxTransducer->attachTaskHandle(trig_left_rx_transducer_task_handle);
        rightRxTransducer->attachTaskHandle(trig_right_rx_transducer_task_handle);
        leftObsDetUS->attachTaskHandle(poll_obs_detection_uss_handle);
        rightObsDetUS->attachTaskHandle(poll_obs_detection_uss_handle);
    }
    log_e("Ultrasonic Subsystem Initialized.");
}

void PeripheralManager::attachBeltInterrupts() {
    int txEcho;
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            txEcho = (int) BeltPin::single_uss_echo;
            log_e("Succesfully Attached Belt Transducer Interrupt (ESP32).");
            break;

        case SocConfig::ESP32_S3_8MB :
            txEcho = (int) S3BeltPin::single_uss_echo;
            log_e("Succesfully Attached Belt Transducer Interrupt (ESP32-S3).");
            break;

        default:
            log_e("Invalid Soc Config. No Valid interrutps.");
            break;
    }

    attachInterruptArg(
        txEcho, 
        on_transducer_us_echo_changed,
        txTransducer, 
        CHANGE
    );
}

void PeripheralManager::attachBotInterrupts() {
    int leftObsEcho, rightObsEcho;
    int leftTransducerEcho, rightTransducerEcho;
    
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            // Attach interrupts for echo pins on 2 rx transducers.
            leftTransducerEcho = (int) BotPin::left_us_transducer_echo;
            rightTransducerEcho = (int) BotPin::right_us_transducer_echo;
            leftObsEcho = (int) BotPin::left_hcsr04_echo;
            rightObsEcho = (int) BotPin::right_hcsr04_echo; 
            log_e("Succesfully Attached Bot Transducer Interrupt (ESP32).");
            break;

        case SocConfig::ESP32_S3_8MB :
            leftTransducerEcho = (int) S3BotPin::left_us_transducer_echo;
            rightTransducerEcho = (int) S3BotPin::right_us_transducer_echo;
            leftObsEcho = (int) S3BotPin::left_hcsr04_echo;
            rightObsEcho = (int) S3BotPin::right_hcsr04_echo;
            log_e("Succesfully Attached Bot Transducer Interrupt (ESP32-S3).");
            break;

        default:
            log_e("Invalid Soc Config. No Valid interrutps.");
            break;
    }

    attachInterruptArg(
        leftTransducerEcho, 
        on_transducer_us_echo_changed, 
        this->leftRxTransducer, 
        CHANGE
    );

    attachInterruptArg(
        rightTransducerEcho, 
        on_transducer_us_echo_changed, 
        this->rightRxTransducer, 
        CHANGE
    );

    // Attach interrupts fro echo pins on 2 obstacle detection HC-SR04s.
    attachInterruptArg(
        leftObsEcho, 
        on_hcsr04_us_echo_changed, 
        this->leftObsDetUS, 
        CHANGE
    );

    attachInterruptArg(
        rightObsEcho, 
        on_hcsr04_us_echo_changed, 
        this->rightObsDetUS, 
        CHANGE
    );
}

// Per name.
void PeripheralManager::beginTasks() {

    // Create and begin all sensor based tasks.
    BaseType_t taskCreated;

    taskCreated = beginTransducerTriggerTasks();
    if(taskCreated != pdPASS) log_e("Transducer trigger tasks not created. Fail Code: %d\n", taskCreated);
    else log_e("Transducer trigger tasks created.");

    if(this->dev->isTransmitter() == false) {
        taskCreated = beginPollObstacleDetectionUssTask();
        if(taskCreated != pdPASS) log_e("Read Ultrasonic Sensor task not created. Fail Code: %d\n", taskCreated);
        else log_e("Read Ultrasonic Sensor task created.");
    }
    
}

bool PeripheralManager::isTransmitter() { return dev->isTransmitter(); }

HCSR04* PeripheralManager::fetchUS(SensorID id) {
    HCSR04 *res = NULL;
    switch (id) {
        case (SensorID::txTransducer):
            res = this->txTransducer;
            break;

        case (SensorID::leftRxTransducer):
            res = this->leftRxTransducer;
            break;

        case (SensorID::rightRxTransducer):
            res = this->rightRxTransducer;
            break;

        case (SensorID::leftObsDet) : 
            res = this->leftObsDetUS;
            break;

        case (SensorID::rightObsDet) : 
            res = this->rightObsDetUS;
            break;

    }
    return res;
}

// Create the task trigger the distance sensing ultrasonic transducer.
BaseType_t PeripheralManager::beginTriggerTxTransducerTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &trig_tx_transducer_task,               // Pointer to task function.
        "trigger_tx_transducer_Task",           // Task name.
        TaskStackDepth::tsd_TRIG,               // Size of stack allocated to the task (in bytes).
        this,                                   // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,            // Task priority level.
        &trig_tx_transducer_task_handle,        // Pointer to task handle.
        1                                       // Core that the task will run on.
    );
    return res;
}

// Create the task trigger the distance sensing ultrasonic transducer.
BaseType_t PeripheralManager::beginTriggerLeftRxTransducerTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &trig_left_rx_transducer_task,          // Pointer to task function.
        "trigger_left_rx_transducer_Task",      // Task name.
        TaskStackDepth::tsd_TRIG,               // Size of stack allocated to the task (in bytes).
        this,                                   // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,            // Task priority level.
        &trig_left_rx_transducer_task_handle,   // Pointer to task handle.
        1                                       // Core that the task will run on.
    );
    return res;
}

// Create the task trigger the distance sensing ultrasonic transducer.
BaseType_t PeripheralManager::beginTriggerRightRxTransducerTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &trig_right_rx_transducer_task,         // Pointer to task function.
        "trigger_right_rx_transducer_task",     // Task name.
        TaskStackDepth::tsd_TRIG,               // Size of stack allocated to the task (in bytes).
        this,                                   // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,            // Task priority level.
        &trig_right_rx_transducer_task_handle,  // Pointer to task handle.
        1                                       // Core that the task will run on.
    );
    return res;
}

BaseType_t PeripheralManager::beginTransducerTriggerTasks() {
    BaseType_t res, res2;
    if(this->isTransmitter()) {
        res = beginTriggerTxTransducerTask();
        if(res != pdPASS) log_e("Tx Trigger Task not created.");
        return res;
    }
    else {
        if(TESTING_LEFT_RX_ONLY == true) {
            res = beginTriggerLeftRxTransducerTask();
            if(res != pdPASS) log_e("Left Rx Trigger Task not created.");
            return res;
        }
        if(TESTING_RIGHT_RX_ONLY == true) {
            res = beginTriggerRightRxTransducerTask();
            if(res != pdPASS) log_e("Right Rx Trigger Task not created.");
            return res;
        }
        res = beginTriggerLeftRxTransducerTask();
        res2 = beginTriggerRightRxTransducerTask();
        if(res != pdPASS) log_e("Left Rx Trigger Task not created.");
        if(res2 != pdPASS) log_e("Right Rx Trigger Task not created.");
        return res && res2;
    }
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
    // Identify the trigger and echo pins based on configuration.
    int trig, echo;
    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            trig = (int) BeltPin::single_uss_trig;
            echo = (int) BeltPin::single_uss_echo;
            break;

        case SocConfig::ESP32_S3_8MB :
            trig = (int) S3BeltPin::single_uss_trig;
            echo = (int) S3BeltPin::single_uss_echo;
            break;

        default:
            trig = -1;
            echo = -1;
            log_e("Invalid Soc Config. Bad Belt Construction.");
            break;
    }

    // Construct the belt peripheral.
    this->txTransducer = new HCSR04(
        trig, 
        echo, 
        SensorID::txTransducer, 
        OBS_LIM,
        T_US_READY
    );
}

void PeripheralManager::constructBotPeripherals() {
    int leftObsTrig, rightObsTrig, leftTransducerTrig, rightTransducerTrig;
    int leftObsEcho, rightObsEcho, leftTransducerEcho, rightTransducerEcho;
    int leftMotLeftPWM, leftMotRightPWM, rightMotLeftPWM, rightMotRightPWM;

    switch (dev->getSocInUse()) {
        case SocConfig::ESP32_4MB :
            leftTransducerTrig = (int) BotPin::left_us_transducer_trig;
            leftTransducerEcho= (int) BotPin::left_us_transducer_echo;
            rightTransducerTrig = (int) BotPin::right_us_transducer_trig;
            rightTransducerEcho= (int) BotPin::right_us_transducer_echo;

            leftObsTrig = (int) BotPin::left_hcsr04_trig;
            leftObsEcho = (int) BotPin::left_hcsr04_echo;
            rightObsTrig = (int) BotPin::right_hcsr04_trig;
            rightObsEcho = (int) BotPin::right_hcsr04_echo;

            leftMotLeftPWM = (int) BotPin::left_mot_left_pwm;
            leftMotRightPWM = (int) BotPin::left_mot_right_pwm;
            rightMotLeftPWM = (int) BotPin::right_mot_left_pwm;
            rightMotRightPWM = (int) BotPin::right_mot_right_pwm;
            break;

        case SocConfig::ESP32_S3_8MB :
            leftTransducerTrig = (int) S3BotPin::left_us_transducer_trig;
            leftTransducerEcho= (int) S3BotPin::left_us_transducer_echo;
            rightTransducerTrig = (int) S3BotPin::right_us_transducer_trig;
            rightTransducerEcho= (int) S3BotPin::right_us_transducer_echo;

            leftObsTrig = (int) S3BotPin::left_hcsr04_trig;
            leftObsEcho = (int) S3BotPin::left_hcsr04_echo;
            rightObsTrig = (int) S3BotPin::right_hcsr04_trig;
            rightObsEcho = (int) S3BotPin::right_hcsr04_echo;

            leftMotLeftPWM = (int) S3BotPin::left_mot_left_pwm;
            leftMotRightPWM = (int) S3BotPin::left_mot_right_pwm;
            rightMotLeftPWM = (int) S3BotPin::right_mot_left_pwm;
            rightMotRightPWM = (int) S3BotPin::right_mot_right_pwm;
            break;

        default:
            Serial.println("Invalid Soc Config. Bad Bot Construction.");
            break;
    }

    // Construct Bot peripherals.
    this->leftRxTransducer = new HCSR04(
        leftTransducerTrig,
        leftTransducerEcho,
        SensorID::leftRxTransducer,
        OBS_LIM,
        T_US_READY
    );

    this->rightRxTransducer = new HCSR04(
        rightTransducerTrig,
        rightTransducerEcho,
        SensorID::rightRxTransducer,
        OBS_LIM,
        T_US_READY
    );

    this->rightObsDetUS = new HCSR04(
        rightObsTrig,
        rightObsEcho,
        SensorID::rightObsDet,
        OBS_LIM,
        R_US_READY
    );

    this->leftObsDetUS = new HCSR04(
        leftObsTrig,
        leftObsEcho,
        SensorID::leftObsDet,
        OBS_LIM,
        L_US_READY
    );

    driveSystem = new BTS7960(
        leftMotLeftPWM,
        leftMotRightPWM,
        rightMotLeftPWM,
        rightMotRightPWM
    );
}

void PeripheralManager::initDriveSystem() {
    driveSystem->init();
}

BaseType_t PeripheralManager::beginDriveTask() {
    BaseType_t res;
    return res;
}