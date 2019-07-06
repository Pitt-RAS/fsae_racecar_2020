#include "Rate.h"
#include <Arduino.h>

Rate::Rate(int hz) {
    SetRate(hz);
    last_us_ = 0;
}

void Rate::SetRate(int hz) {
    period_us_ = (1.0 / (double)hz) * 1000000;
}

bool Rate::NeedsRun() {
    unsigned long dt = micros() - last_us_;
    if ( dt >= period_us_ ) {
        last_us_ = micros();
        return true;
    }
    return false;
}

void Rate::Sleep() {
    unsigned long dt = micros() - last_us_;

    if ( dt < period_us_ )
        delayMicroseconds(period_us_ - dt);

    last_us_ = micros();
}
