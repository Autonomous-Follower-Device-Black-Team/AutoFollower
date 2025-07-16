#include "BTS7960.h"

void BTS7960::init() {
    leftMotors.init();
    rightMotors.init();
}

void BTS7960::setSpeed(int leftSideSpeed, int rightSideSpeed) {
    leftMotors.setSpeed(leftSideSpeed);
    rightMotors.setSpeed(rightSideSpeed);
}

void BTS7960::move(int leftSpeed, int rightSpeed) {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }
    this->setSpeed(leftSpeed, rightSpeed);
    leftMotors.spinCCW();
    rightMotors.spinCW();
}

void BTS7960::moveForward(int speed) {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }
    this->setSpeed(speed, speed);
    leftMotors.spinCCW();
    rightMotors.spinCW();
}

void BTS7960::moveBackward(int speed) {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }

    this->setSpeed(speed, speed);
    leftMotors.spinCW();
    rightMotors.spinCCW();
}

void BTS7960::rotateLeft() {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }

    leftMotors.spinCW();
    rightMotors.spinCW();
}

void BTS7960::rotateRight() {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }

    leftMotors.spinCCW();
    rightMotors.spinCCW();
}

void BTS7960::turnLeft(int speedOffset) {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }

    int leftSpeed = BASE_SPEED - speedOffset;
    int rightSpeed = BASE_SPEED + speedOffset;
    this->setSpeed(leftSpeed, rightSpeed);
    leftMotors.spinCW();
    rightMotors.spinCCW(); 
}

void BTS7960::turnRight(int speedOffset) {
    if(driveStatus == MovementState::EMG_STOP) {
        log_e("Errant Move Attempt. Stop Active.");
        return;
    }

    int leftSpeed = BASE_SPEED - speedOffset;
    int rightSpeed = BASE_SPEED + speedOffset;
    this->setSpeed(leftSpeed, rightSpeed);
    leftMotors.spinCCW();
    rightMotors.spinCW();
}

void BTS7960::stop() {
    leftMotors.stop(stopType::COAST);
    rightMotors.stop(stopType::COAST);
}

MovementState BTS7960::getMovementState() {
    return driveStatus;
}

void BTS7960::setMovementState(MovementState status) {
    this->driveStatus = status;
}