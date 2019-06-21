#include "SoftWatchdog.h"
#include <Arduino.h>

SoftWatchdog::SoftWatchdog(unsigned int max_age_ms) {
    last_fed_ = 0;
    max_age_ = max_age_ms;
}

void SoftWatchdog::Feed() {
    last_fed_ = millis();
}

bool SoftWatchdog::Hungry() {
    return millis() - last_fed_ >= max_age_;
}
