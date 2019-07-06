#ifndef HEARTBEAT_LED_H
#define HEARTBEAT_LED_H

#include "config.h"
#include "RobotState.h"
#include "Rate.h"

class HeartbeatLED {
public:
    HeartbeatLED();
    void SetState(RobotState state);
    void Update();
private:
    Rate heartbeat_led_rate_;
    bool heartbeat_led_state_;
};

#endif
