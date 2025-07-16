#ifndef BTS7960_H
#define BTS7960_H

#include <Arduino.h>
#include "Motor.h"

enum _mvmt_state {
    EMG_STOP,
    PAUSED,
    MOVING
};
typedef enum _mvmt_state MovementState;

class BTS7960 {
    private:
        Motor leftMotors;
        Motor rightMotors;
        MovementState driveStatus = MovementState::PAUSED;

        void setSpeed(int leftSideSpeed = BASE_SPEED, int rightSideSpeed = BASE_SPEED);

    public:
        BTS7960(int leftPwmL, int leftPwmR, int rightPwmL, int rightPwmR) : 
            leftMotors(leftPwmL, leftPwmR),
            rightMotors(rightPwmL, rightPwmR) {}
        
        void init();
        void move(int leftSpeed, int rightSpeed);
        void moveForward(int speed);
        void moveBackward(int speed);
        void rotateLeft();
        void rotateRight();
        void turnLeft(int speedOffset);
        void turnRight(int speedOffset);
        void stop();

        MovementState getMovementState();
        void setMovementState(MovementState status);

};


#endif /* BTS7960_H */