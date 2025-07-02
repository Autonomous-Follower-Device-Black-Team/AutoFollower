#include "BTS7960.h"

void BTS7960::init() {
    leftMotors.init();
    rightMotors.init();
}

void BTS7960::setSpeed(int leftSideSpeed, int rightSideSpeed) {
    leftMotors.setSpeed(leftSideSpeed);
    rightMotors.setSpeed(rightSideSpeed);
}

void BTS7960::moveForward(int speed) {
    this->setSpeed(speed, speed);
    leftMotors.spinCCW();
    rightMotors.spinCW();
}

void BTS7960::moveBackward(int speed) {
    this->setSpeed(speed, speed);
    leftMotors.spinCW();
    rightMotors.spinCCW();
}

void BTS7960::rotateLeft() {
    leftMotors.spinCW();
    rightMotors.spinCW();
}

void BTS7960::rotateRight() {
    leftMotors.spinCCW();
    rightMotors.spinCCW();
}

void BTS7960::turnLeft(int speedOffset) {
    int leftSpeed = BASE_SPEED - speedOffset;
    int rightSpeed = BASE_SPEED + speedOffset;
    this->setSpeed(leftSpeed, rightSpeed);
    leftMotors.spinCW();
    rightMotors.spinCCW(); 
}

void BTS7960::turnRight(int speedOffset) {
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

bool BTS7960::getStopStatus() {
    return emergencyStop;
}

void BTS7960::setStopStatus(bool status) {
    this->emergencyStop = status;
}