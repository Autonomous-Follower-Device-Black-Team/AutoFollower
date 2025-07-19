#include "PeripheralManager.h"
#include "Device.h"

// Define Event Group and Mutex Handles.
EventGroupHandle_t rx_trig_sync_group = NULL;
EventGroupHandle_t rx_echo_time_group = NULL;
SemaphoreHandle_t rx_echo_time_mutex = NULL;
SemaphoreHandle_t echo_diff_buffer_mutex = NULL;
SemaphoreHandle_t drive_system_mutex = NULL;

// Define task handles.
TaskHandle_t trig_tx_transducer_task_handle = NULL;  
TaskHandle_t trig_left_rx_transducer_task_handle = NULL;
TaskHandle_t trig_right_rx_transducer_task_handle = NULL;
TaskHandle_t poll_obs_detection_uss_handle = NULL;      
TaskHandle_t obs_det_stop_task_handle = NULL;
TaskHandle_t mvmt_manager_task_handle = NULL;    
TaskHandle_t left_right_rx_diff_task_handle = NULL;     

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

        // Wait for notifcation from trigger timer before trigger.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

        // Trigger the transmitter.
        readingGood = transducer->readSensor(US_READ_TIME);

        // Log triggers and errant succesful readings if they exist. 
        // ReadingGood will always be false due to the mutilation of the sensor into tx only.
        if(!readingGood) log_e("Tx triggered Succesfully.");
        else log_e("Tx Trigger Issue.");
    }
}

void trig_rx_transducer_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    HCSR04 *transducer = manager->fetchTransducer(xTaskGetCurrentTaskHandle()); 
    NotificationMask tdTrigBits = (transducer->identify() == SensorID::leftRxTransducer) ? TRIG_L_RX : TRIG_R_RX;
    NotificationMask tdReady = (transducer->identify() == SensorID::leftRxTransducer) ? L_TD_READY : R_TD_READY;
    EventBits_t triggerWatch;
    bool readingGood;

    for(;;) {

        // Block until trigger ready.
        triggerWatch = xEventGroupWaitBits(
            rx_trig_sync_group,     // Event group to watch.
            tdTrigBits,             // Bits to wait for.
            pdTRUE,                 // Clear on exit.
            pdTRUE,                 // Wait for all bits to be set before unblocking.
            portMAX_DELAY           // Wait for the longest amount of time.
        );

        // Trigger transducer.
        readingGood = transducer->readSensor(RX_US_READ_TIME);

        // Grab the semaphore and set the times.
        grabMutex(&rx_echo_time_mutex);
        manager->fillUsTimingGroup(transducer->identify());
        releaseMutex(&rx_echo_time_mutex);

        // Debug if necessary.
        if(RX_DEBUG) {
            if(readingGood) {
                float instDistance = transducer->getDistanceReading() * 2;
                float avgDistance = transducer->getLastBufferAverage() * 2;
                Serial.printf("[%s] Rx: Distance: %f, Average: %f\n", transducer->getName().c_str(), instDistance, avgDistance);
            }
            else log_e("[%d] Rx Failed.", transducer->identify());
        }
    
        // Event: Transducer (left/right) data ready.
        xEventGroupSetBits(rx_echo_time_group, tdReady);
    }
}

void left_right_rx_diff_task(void *pvPeripheralManager) {
    // Setup.
    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    HCSR04 *leftRx = manager->fetchUS(SensorID::leftRxTransducer);
    HCSR04 *rightRx = manager->fetchUS(SensorID::rightRxTransducer);
    USTimeGroup *timingGroup = manager->getUsTimingGroup();
    EventBits_t echoes = -1;
    signed long long echoDifference;
    float avgDistance;
    signed long long lst, let, rst, ret;    

    // Task Loop.
    for(;;) {

        // Block until both echoes are ready. 
        echoes = xEventGroupWaitBits(
            rx_echo_time_group,     // Event group to watch.
            TD_READY,               // Bits to wait for.
            pdTRUE,                 // Clear on exit.
            pdTRUE,                 // Wait for all bits to be set before unblocking.
            portMAX_DELAY           // Wait for the longest amount of time.
        );

        // Skip over unexpected events with a log message.
        if((echoes & TD_READY) != TD_READY) {
            log_e("Unexpected Event: 0x%x", echoes);
            continue;
        }

        // Grab the semaphore and grab the times.
        grabMutex(&rx_echo_time_mutex);
        lst = timingGroup->leftStartTime;
        let = timingGroup->leftEndTime;
        rst = timingGroup->rightStartTime;
        ret = timingGroup->rightEndTime;
        releaseMutex(&rx_echo_time_mutex);
    
        // Dump data if debugging.
        if(DUMP_RX_DIFF) dump_rx_diff_info(lst, rst, let, ret);

        // Compute difference between echo high times (right biased) and avg distance.
        echoDifference = ret - let;
        avgDistance = (leftRx->getDistanceReading() + rightRx->getDistanceReading())/2; 

        // Store difference and distance history.
        grabMutex(&echo_diff_buffer_mutex);
        manager->addToBuffers(echoDifference, avgDistance);
        releaseMutex(&echo_diff_buffer_mutex);

        // Notify the movement manager Task given that data is ready.
        if(manager->isBufferReadyForUse() && RX_DRIVE_SYSTEM_ON) {
            if(mvmt_manager_task_handle != NULL) xTaskNotify(mvmt_manager_task_handle, ECHO_DIFF_READY, eSetBits);
            else log_e("Movement Manager Task not notified. Null.");
        }
        
    }
}

void poll_obs_detection_uss_task(void *pvPeripheralManager) {
    // Initialize task.
    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    HCSR04 *left_obs = manager->fetchUS(SensorID::leftObsDet); 
    HCSR04 *right_obs = manager->fetchUS(SensorID::rightObsDet); 

    // Flags for Obstacle Logic.
    ulong start_time = 0;
    ulong current_time;
    int buzzer = (int) S3BotPin::BUZZER_PIN;
    pinMode(buzzer, OUTPUT);

    bool readingGood_L, readingGood_R;
    float getDistance_L, getDistance_R;

    bool Breach_L;
    bool Breach_R;

    bool buzzer_on = false;

    NotificationMask last_notif = UNSET;

    // Begin task loop.
    for(;;) {
        // Something for the buzzer. (grabs current time).
        current_time = millis();

        // Reads the Obs USS.
        readingGood_L = left_obs->readSensor(US_READ_TIME);
        readingGood_R = right_obs->readSensor(US_READ_TIME);

        // Checks if the reading from Obs USS is good.
        if((readingGood_L && readingGood_R) && RX_DRIVE_SYSTEM_ON){
            //getDistance_L = left_obs->getDistanceReading();
            //getDistance_R = right_obs->getDistanceReading();
            getDistance_L = left_obs->getLastBufferAverage();
            getDistance_R = right_obs->getLastBufferAverage();

            // Checks if obstacle is detected within bound.
            Breach_L = getDistance_L > 0 && getDistance_L <= BREACH_DISTANCE;
            Breach_R = getDistance_R > 0 && getDistance_R <= BREACH_DISTANCE;

            // Turns buzzer off after set time.
            if (buzzer_on && (current_time - start_time > STOP_SCREAMING)){
                digitalWrite(buzzer, LOW);
                buzzer_on = false;
                log_e("Buzzer shuts up");
            }

            // Notifies that an obstacle has been detected.
            if(Breach_L || Breach_R) {
                if (obs_det_stop_task_handle != NULL && last_notif != MOT_E_STOP) {
                    start_time = current_time;
                    buzzer_on = true;
                    digitalWrite(buzzer, HIGH);
                    xTaskNotify(obs_det_stop_task_handle, MOT_E_STOP, eSetBits);
                    last_notif = MOT_E_STOP;
                    log_e("Obstacle detected & buzzer activated");
                }
                else {
                    if(obs_det_stop_task_handle == NULL) log_e("Obstacle Detection Manager Task not notified. Null.");
                    else log_e("Obstacle Detection Manager Task not notified. E_STOP Sent Once Before.");
                }
            }

            // Notifies that an obstacle is removed and motors can resume.
            else {
                if (obs_det_stop_task_handle != NULL && last_notif != MOT_RESUME) {
                    if(last_notif != MOT_RESUME) {
                        xTaskNotify(obs_det_stop_task_handle, MOT_RESUME, eSetBits); 
                        last_notif = MOT_RESUME;
                        log_e("Obstacle not detected, moving resumes");
                    }
                }
                else {
                    if(obs_det_stop_task_handle == NULL) log_e("Obstacle Detection Manager Task not notified. Null.");
                    else log_e("Obstacle Detection Manager Task not notified. E_STOP Sent Once Before.");
                }
            }
        }
        else {
            if(buzzer_on && (!readingGood_L && !readingGood_R)){
                digitalWrite(buzzer, LOW);
                buzzer_on = false;
                log_e("Buzzer shuts up because of invalid reading");
            }
            if(!RX_DRIVE_SYSTEM_ON) log_e("Drive System Off. No Notification sent.");
        }
        vTaskDelayUntil(&xLastWakeTime, MAX_US_POLL_TIME + pdMS_TO_TICKS(10));
    }
}

void obs_det_stop_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    BTS7960 *driveSystem = manager->getDriveSystem();

    // Begin task loop.
    for(;;) {

        // Wait for notifcation from obstacle detection task.
        uint32_t emergencyStopEvent = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        grabMutex(&drive_system_mutex);
        switch (emergencyStopEvent) {
            case (MOT_E_STOP):
                driveSystem->stop();
                driveSystem->setMovementState(MovementState::EMG_STOP);
                log_e("Emergency Stop. Motors Stopped.");
                break;
            
            case (MOT_RESUME):
                driveSystem->setMovementState(MovementState::PAUSED);
                break;

            default:
                log_e("Unhandled Notification. Motors Stopped.");
                break;
        }
        releaseMutex(&drive_system_mutex);
    }
} 

void mvmt_manager_task(void *pvPeripheralManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    PeripheralManager *manager = static_cast<PeripheralManager *>(pvPeripheralManager);
    BTS7960 *driveSystem = manager->getDriveSystem();
    BangBangCtrlConfig *driveCfg = manager->getFollowingLogicConfig();
    BufferAverages rxBuffers;
    uint8_t steeringOffset, distanceOffset;
    uint16_t leftMotSpeed, rightMotSpeed;
    bool validMovePossible = false, steeringOffsetRequired = false;

    for(;;) {
        // Block until notified.
        uint32_t notifValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Ensure correct notificiation.
        if((notifValue & ECHO_DIFF_READY) != ECHO_DIFF_READY) {
            log_e("Mvmt Manager Incorrectly Notified.");
            continue;;
        }

        // Grab new difference and distance data.
        grabMutex(&echo_diff_buffer_mutex);
        rxBuffers = manager->getDiffBufferAverages();
        releaseMutex(&echo_diff_buffer_mutex);
        
        // Compute offsets, speeds, movement status.
        steeringOffsetRequired = rxBuffers.echoDiffAvg < driveCfg->edLower || rxBuffers.echoDiffAvg > driveCfg->edUpper;
        distanceOffset = (uint8_t) (driveCfg->kz * (1 - driveCfg->targetDist/rxBuffers.distAvg));
        steeringOffset = (steeringOffsetRequired) ? (uint8_t) (driveCfg->kp *  rxBuffers.echoDiffAvg) : 0;
       
        leftMotSpeed = driveCfg->defSpeed + distanceOffset + steeringOffset;
        rightMotSpeed = driveCfg->defSpeed + distanceOffset - steeringOffset;
        validMovePossible = (rxBuffers.distAvg > (driveCfg->targetDist * 1.05)) && (rxBuffers.distAvg < driveCfg->maxDist); 

        // Clamp Motor Speeds.
        if(leftMotSpeed > driveCfg->maxSpeed) leftMotSpeed = driveCfg->maxSpeed;
        else if(leftMotSpeed < driveCfg->minSpeed) leftMotSpeed = driveCfg->maxSpeed;

        if(rightMotSpeed > driveCfg->maxSpeed) rightMotSpeed = driveCfg->maxSpeed;
        else if(rightMotSpeed < driveCfg->minSpeed) rightMotSpeed = driveCfg->maxSpeed;

        if(validMovePossible) {
            Serial.printf("wL = %d, wR = %d,\n", leftMotSpeed, rightMotSpeed);
        }

        // Drive.
        grabMutex(&drive_system_mutex);
        
        // Attempt movment only when an obstacle is not present.
        switch(driveSystem->getMovementState()) {
            case (MovementState::EMG_STOP):
                // Do nothing.
                if(RX_DRIVE_DEBUG) log_e("Mvmt State[E_STOP]: Motors Stopped! Obstacle Detected");
                break;

            case (MovementState::PAUSED):
                if(validMovePossible) {
                    driveSystem->move(leftMotSpeed, rightMotSpeed);
                    driveSystem->setMovementState(MovementState::MOVING);
                    if(RX_DRIVE_DEBUG) log_e("Mvmt State[PAUSED]: Valid Move Possible --> Motors set to MOVING.");
                }
                else {
                    if(RX_DRIVE_DEBUG) log_e("Mvmt State[PAUSED]: Valid Move NOT Possible: %f --> Motors remain PAUSED.", rxBuffers.distAvg);
                }
                break;

            case (MovementState::MOVING):
                if(validMovePossible) {
                    driveSystem->move(leftMotSpeed, rightMotSpeed);
                    if(RX_DRIVE_DEBUG) log_e("Mvmt State[MOVING]: Valid Move Possible --> Motors remain MOVING.");
                }
                else {
                    driveSystem->stop();
                    driveSystem->setMovementState(MovementState::PAUSED);
                    if(RX_DRIVE_DEBUG) log_e("Mvmt State[MOVING]: Valid Move NOT Possible: %f --> Motors set to PAUSED.", rxBuffers.distAvg);
                }   
                break;
                
            default:
                if(RX_DRIVE_DEBUG) log_e("Errant Following Movement State...");
                break;
        }
        
        releaseMutex(&drive_system_mutex);
    

        // Print avg.
        //Serial.printf("%f\n", avgEchoDiff);

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

/**
 * Initialze the Ultrasonic Sensors.
 */
void PeripheralManager::initUS() {

    // Initialize the ultrasonic sensors.
    if(dev->isTransmitter()) {
        txTransducer->init();
        txTransducer->attachTaskHandle(&trig_tx_transducer_task_handle);
    }
    else {
        leftRxTransducer->init();
        rightRxTransducer->init();
        leftObsDetUS->init();
        rightObsDetUS->init();

        this->initRxHistoryBuffers();

        leftRxTransducer->attachTaskHandle(&trig_left_rx_transducer_task_handle);
        rightRxTransducer->attachTaskHandle(&trig_right_rx_transducer_task_handle);
        leftObsDetUS->attachTaskHandle(&poll_obs_detection_uss_handle);
        rightObsDetUS->attachTaskHandle(&poll_obs_detection_uss_handle);

    }
    log_e("Ultrasonic Subsystem Initialized.");
}

void PeripheralManager::createSemaphores() {
    bool success;

    // Create transmitter semaphores.
    if(this->dev->isTransmitter()) {
        // Currently no Transmitter semaphores.
    }

    // Create receiver semaphores.
    else {
        if(RX_ULTRASONIC_SYSTEM_ON) {
            //rx_echo_time_mutex = xSemaphoreCreateMutex();
            success = initMutex(&rx_echo_time_mutex);
            if(!success) log_e("Echo Timing Mutex not created");
            else log_e("Echo Timing Mutex created.");

            //echo_diff_buffer_mutex = xSemaphoreCreateMutex();
            success = initMutex(&echo_diff_buffer_mutex);
            if(!success) log_e("Echo Difference History Mutex not created.");
            else log_e("Echo Difference History Mutex created.");
        }

        if(RX_DRIVE_SYSTEM_ON) {
            success = initMutex(&drive_system_mutex);
            if(!success) log_e("Drive System Mutex not created.");
            else log_e("Drive System Mutex created.");
        }
    }
}

void PeripheralManager::createEventGroups() {
    
    // Create transmitter event groups.
    if(this->dev->isTransmitter()) {
        // Currently no transmitter event groups.
    }

    // Create receiver event groups.
    else {
        if(RX_ULTRASONIC_SYSTEM_ON) {
            rx_echo_time_group = xEventGroupCreate();
            if(rx_echo_time_group == NULL) log_e("Echo Timing event group not created.");
            else log_e("Echo Timing event group created.");

            rx_trig_sync_group = xEventGroupCreate();
            if(rx_trig_sync_group == NULL) log_e("Trigger Syncing event group not created.");
            else log_e("Trigger Syncing event group created.");
        }
        else log_e("Rx Ultrasonic Subsystem Event Groups not created. Check config.");
    }
}

// Per name.
void PeripheralManager::beginTasks() {

    // Create and begin all sensor based tasks.
    BaseType_t taskCreated;

    // Start the transmitter tasks.
    if(this->dev->isTransmitter()) {
        if(TX_ULTRASONIC_SYSTEM_ON) {
            taskCreated = beginTransducerTriggerTasks();
            if(taskCreated != pdPASS) log_e("Transducer trigger tasks not created. Fail Code: %d\n", taskCreated);
            else log_e("Transducer trigger tasks created.");
        }
        else log_e("Tx Ultrasonic Subsytem not started. Check config.");
    }   

    // Start the receiver tasks.
    else if(this->dev->isTransmitter() == false) {

        // Start the ultrasonic subsystem.
        if(RX_ULTRASONIC_SYSTEM_ON) {

            taskCreated = beginTransducerTriggerTasks();
            if(taskCreated != pdPASS) log_e("Transducer trigger tasks not created. Fail Code: %d\n", taskCreated);
            else log_e("Transducer trigger tasks created.");

            taskCreated = beginReceiverEchoDiffTask();
            if(taskCreated != pdPASS) log_e("Echo time difference task not created. Fail Code: %d\n", taskCreated);
            else log_e("Echo time difference task created.");

            taskCreated = beginPollObstacleDetectionUssTask();
            if(taskCreated != pdPASS) log_e("Poll Obstacle Detection USS task not created. Fail Code: %d\n", taskCreated);
            else log_e("Poll Obstacle Detection USS task created.");
        }
        else log_e("Rx Ultrasonic Subsystem Tasks not started. Check config.");
        
        // Start the Drive System.
        if(RX_DRIVE_SYSTEM_ON) {
            taskCreated = beginEmergencyStopTask();
            if(taskCreated != pdPASS) log_e("Emergency Stop task not created. Fail Code: %d\n", taskCreated);
            else log_e("Emergency Stop task created.");

            taskCreated = beginMovementManagerTask();
            if(taskCreated != pdPASS) log_e("Movement Manager task not created. Fail Code: %d\n", taskCreated);
            else log_e("Movement Manager task created.");
        }
        else log_e("Rx Drive System Subsystem not started. Check config.");
    }
    
}

bool PeripheralManager::isTransmitter() { return dev->isTransmitter(); }

HCSR04 *PeripheralManager::fetchUS(SensorID id) {
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

HCSR04 *PeripheralManager::fetchTransducer(TaskHandle_t handle) {
    HCSR04 *res = NULL;
    if(handle == leftRxTransducer->getTaskHandle()) res = leftRxTransducer;
    else if(handle == rightRxTransducer->getTaskHandle()) res = rightRxTransducer;
    return res;
}

void PeripheralManager::fillUsTimingGroup(SensorID id) {
    ulong st = fetchUS(id)->getISRStartPulse();
    ulong et = fetchUS(id)->getISREndPulse();
    switch(id) {
        case (SensorID::leftRxTransducer) :
            usTimingGroup.leftStartTime = (signed long long) st;
            usTimingGroup.leftEndTime = (signed long long) et;
            break;
            
        case (SensorID::rightRxTransducer) :
            usTimingGroup.rightStartTime = (signed long long) st;
            usTimingGroup.rightEndTime = (signed long long) et;
            break;
    }
}

USTimeGroup *PeripheralManager::getUsTimingGroup() { return &this->usTimingGroup; }

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
        &trig_rx_transducer_task,               // Pointer to task function.
        "trig_l_rx_td",                         // Task name.
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
        &trig_rx_transducer_task,               // Pointer to task function.
        "trig_r_rx_td",                         // Task name.
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
            else log_e("Left Rx Trigger Task created.");
            return res;
        }
        if(TESTING_RIGHT_RX_ONLY == true) {
            res = beginTriggerRightRxTransducerTask();
            if(res != pdPASS) log_e("Right Rx Trigger Task not created.");
            else log_e("Right Rx Trigger Task created.");
            return res;
        }
        res = beginTriggerLeftRxTransducerTask();
        res2 = beginTriggerRightRxTransducerTask();
        if(res != pdPASS) log_e("Left Rx Trigger Task not created.");
        else log_e("Left Rx Trigger Task created.");

        if(res2 != pdPASS) log_e("Right Rx Trigger Task not created.");
        else log_e("Right Rx Trigger Task created.");
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

BaseType_t PeripheralManager::beginReceiverEchoDiffTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &left_right_rx_diff_task,           // Pointer to task function.
        "left_right_rx_diff_task",          // Task name.
        TaskStackDepth::tsd_POLL,           // Size of stack allocated to the task (in bytes).
        this,                               // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,        // Task priority level.
        &left_right_rx_diff_task_handle,    // Pointer to task handle.
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
        L_TD_VALID,
        !L_TD_VALID
    );

    this->rightRxTransducer = new HCSR04(
        rightTransducerTrig,
        rightTransducerEcho,
        SensorID::rightRxTransducer,
        OBS_LIM,
        R_TD_VALID,
        !R_TD_VALID
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
    if(this->isTransmitter() == false) {
        this->driveSystem->init();
        this->initBangBangCtrlConfig();
        log_e("Drive Subsystem Initialized.");
    }
}

BaseType_t PeripheralManager::beginEmergencyStopTask()  {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &obs_det_stop_task,             // Pointer to task function.
        "emergency_stop_task",          // Task name.
        TaskStackDepth::tsd_SET,        // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &obs_det_stop_task_handle,      // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

BaseType_t PeripheralManager::beginMovementManagerTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &mvmt_manager_task,             // Pointer to task function.
        "mvmt_manger_task",             // Task name.
        TaskStackDepth::tsd_DRIVE,      // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &mvmt_manager_task_handle,      // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

BTS7960 *PeripheralManager::getDriveSystem() { return this->driveSystem; }

void PeripheralManager::initRxHistoryBuffers(int size) {
    rxHistoryBuffers.readyForUse = false;
    rxHistoryBuffers.size = size;
    rxHistoryBuffers.index = 0;
    rxHistoryBuffers.echoBuffer = (signed long long *) malloc(sizeof(signed long long) * size);
    rxHistoryBuffers.distBuffer = (float *) malloc(sizeof(float) * size);
}

void PeripheralManager::addToBuffers(signed long long echoDiff, float avgDist) {
    if(rxHistoryBuffers.index == rxHistoryBuffers.size) {
        rxHistoryBuffers.index = 0;
        if(!rxHistoryBuffers.readyForUse) 
            rxHistoryBuffers.readyForUse = true;
    }
    rxHistoryBuffers.echoBuffer[rxHistoryBuffers.index] = echoDiff;
    rxHistoryBuffers.distBuffer[rxHistoryBuffers.index] = avgDist;
    rxHistoryBuffers.index++;
}

BufferAverages PeripheralManager::getDiffBufferAverages() {
    BufferAverages res = {BUF_INV, BUF_INV};
    // Return error if buffer not ready for use.
    if(!rxHistoryBuffers.readyForUse) return res;

    // Take the averages and return.
    signed long long echoSum = 0;
    float distSum = 0;

    for(int i = 0; i < rxHistoryBuffers.size; i++) {
        echoSum += rxHistoryBuffers.echoBuffer[i];
        distSum += rxHistoryBuffers.distBuffer[i];
    }
    
    res.echoDiffAvg = ((float) echoSum)/((float) rxHistoryBuffers.size);
    res.distAvg = ((float) distSum)/((float) rxHistoryBuffers.size);

    return res;
}

bool PeripheralManager::isBufferReadyForUse() {
    return rxHistoryBuffers.readyForUse;
}

void PeripheralManager::initBangBangCtrlConfig(){
    followingLogicConfig.minSpeed = MIN_SPEED;
    followingLogicConfig.maxSpeed = MAX_SPEED;
    followingLogicConfig.defSpeed = DEFAULT_SPEED;
    followingLogicConfig.targetDist = TARGET_DIST_IN;
    followingLogicConfig.maxDist = MAX__FOLLOW_DIST_IN;
    followingLogicConfig.edLower = ECHO_DIFF_LOWER_BOUND;
    followingLogicConfig.edUpper = ECHO_DIFF_UPPER_BOUND;
    followingLogicConfig.kp = DEFAULT_KP;
    followingLogicConfig.kz = DEFAULT_KZ;
}

void PeripheralManager::setFollowingLogicDifferentialGain(uint8_t k) {
    this->followingLogicConfig.kp = k;
}

void PeripheralManager::setFollowingLogicDistanceGain(uint8_t k) {
    this->followingLogicConfig.kz = k;
}

BangBangCtrlConfig *PeripheralManager::getFollowingLogicConfig() {
    return &(this->followingLogicConfig);
}

void dump_rx_diff_info(signed long long lst, signed long long rst, signed long long let, signed long long ret) {
    static int count = 0;

    // Print header.
    if(count == 0) {
        Serial.printf("Count; L_t_end; L_t_sta; R_t_end; R_t_sta; L_echo_dur; R_echo_dur; echo_dur_diff(R-L); echo_end_diff(R-L); L_dist; R_dist\n");
    }

    // Print data.
    signed long long echoEndDiff = ret - let;
    bool diffPosAndLarger = (echoEndDiff > 0) && (echoEndDiff >= RX_DIFF_INVALID);
    bool diffNegAndSmaller = (echoEndDiff < 0) && (echoEndDiff <= -1*RX_DIFF_INVALID);
    //if(diffPosAndLarger) echoEndDiff = RX_DIFF_INVALID;
    //else if(diffNegAndSmaller) echoEndDiff = -1*RX_DIFF_INVALID;

    Serial.printf("%d; %lld; %lld; %lld; %lld; %lld; %lld; %lld; %lld; %f; %f\n", 
        count++,                        // Counter.
        let,                            // Left End Time
        lst,                            // Left Start Time
        ret,                            // Right End Time
        rst,                            // Right Start Time
        let - lst,                      // Left Echo High Time
        ret - rst,                      // Right Echo High Time
        (ret - rst) - (let - lst),      // Right Echo Dur - Left Echo Dur
        echoEndDiff,                    // Right Echo End - Left Echo End
        ((float)(let - lst))/74.0,      // Left Distance Recorded
        ((float)(ret - rst))/74.0       // Right Distance Recorded
    );
}

/**
 * Initialize a mutex. 
 * @param mutexPtr Pointer to the mutex to be initialized.
 */
bool initMutex(SemaphoreHandle_t *mutexPtr) {
    bool res = false;
    if(mutexPtr == NULL) log_e("Null function argument.");
    else {
        *mutexPtr = xSemaphoreCreateMutex();
        res = *mutexPtr != NULL;
    }
    return res;
}

/**
 * Grabs a mutex. This function will block the task it's called from
 * for a certain amount of time in ticks with the default time being 
 * the maximum allowable by the system unless otherwise specified.
 * @param mutexPtr Pointer to the mutex to grab.
 * @param timeout Timeout (in ticks) to wait try and grab the mutex. Defaults to 
 * `portMAX_DELAY`.
 */
BaseType_t grabMutex(SemaphoreHandle_t *mutexPtr, TickType_t timeout) {
    BaseType_t res = pdFAIL;
    if(mutexPtr != NULL && *mutexPtr != NULL) {
        res = xSemaphoreTake(*mutexPtr, timeout);
    }
    else {
        if(mutexPtr == NULL) log_e("Null function argument.");
        if(*mutexPtr == NULL) log_e("Function argument points to Null.");
    }
    return res;
}

/**
 * Releases a mutex. 
 * @param mutexPtr Pointer to the mutex to release.
 */
BaseType_t releaseMutex(SemaphoreHandle_t *mutexPtr) {
    BaseType_t res = pdFAIL;
    if(mutexPtr != NULL && *mutexPtr != NULL) {
        res = xSemaphoreGive(*mutexPtr);
    }
    else {
        if(mutexPtr == NULL) log_e("Null function argument.");
        if(*mutexPtr == NULL) log_e("Function argument points to Null.");
    }
    return res;
}