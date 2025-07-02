#ifndef BTS7960_H
#define BTS7960_H

#include <Arduino.h>
#include "Motor.h"

class BTS7960 {
    private:
        Motor leftMotors;
        Motor rightMotors;
        bool emergencyStop = false;

        void setSpeed(int leftSideSpeed = BASE_SPEED, int rightSideSpeed = BASE_SPEED);

    public:
        BTS7960(int leftPwmL, int leftPwmR, int rightPwmL, int rightPwmR) : 
            leftMotors(leftPwmL, leftPwmR),
            rightMotors(rightPwmL, rightPwmR) {}
        
        void init();
        void moveForward(int speed);
        void moveBackward(int speed);
        void rotateLeft();
        void rotateRight();
        void turnLeft(int speedOffset);
        void turnRight(int speedOffset);
        void stop();

        bool getStopStatus();
        void setStopStatus(bool status);

};


#endif /* BTS7960_H */