#ifndef BTS7960_H
#define BTS7960_H

#include "Motor.h"

class BTS7960 {
    private:
        Motor leftMotors;
        Motor rightMotors;

    public:
        BTS7960(int leftPwmL, int leftPwmR, int rightPwmL, int rightPwmR) : 
            leftMotors(leftPwmL, leftPwmR),
            rightMotors(rightPwmL, rightPwmR) {}
        
        void init();

};


#endif /* BTS7960_H */