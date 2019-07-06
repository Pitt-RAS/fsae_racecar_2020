#ifndef PWM_H
#define PWM_H

#include <Servo.h>

class PWM {
public:
    PWM(int pin);
    void Set(double speed);
    void ConfigHighLimit(double limit);
    void ConfigLowLimit(double limit);
    void ConfigOffset(double offset);
private:
    Servo pwm_;
    double low_limit_;
    double high_limit_;
    double offset_;
};

#endif
