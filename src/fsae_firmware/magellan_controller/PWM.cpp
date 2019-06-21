#include "PWM.h"
#include <math.h>

PWM::PWM(int pin) {
    pwm_.attach(pin);
    ConfigHighLimit(1);
    ConfigLowLimit(0);
    ConfigOffset(0);
    Set(0);
}

void PWM::ConfigHighLimit(double limit) {
    high_limit_ = limit;
}

void PWM::ConfigLowLimit(double limit) {
    low_limit_ = limit;
}

void PWM::ConfigOffset(double offset) {
    offset_ = offset;
}

void PWM::Set(double speed) {
    if ( fabs(speed) > high_limit_ )
        speed = high_limit_;
    else if ( fabs(speed) < low_limit_ )
        speed = 0;

    speed += offset_;

    // Force bounds to valid PWM range
    if ( fabs(speed) > 1 )
        speed = copysign(1, speed);

    speed *= 500;
    speed += 1500;

    pwm_.writeMicroseconds((int)speed);
}
