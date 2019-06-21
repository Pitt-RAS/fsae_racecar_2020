#include "HeartbeatLED.h"
#include "Arduino.h"

HeartbeatLED::HeartbeatLED() :
        heartbeat_led_rate_(HEARTBEAT_DISABLED_HZ),
        heartbeat_led_state_(false) {
    pinMode(HEARTBEAT_LED, OUTPUT);
}

void HeartbeatLED::SetState(RobotState state) {
    if ( state == MODE_DISABLED )
        heartbeat_led_rate_.SetRate(HEARTBEAT_DISABLED_HZ);
    else if ( state == MODE_TELEOP )
        heartbeat_led_rate_.SetRate(HEARTBEAT_TELEOP_HZ);
    else if ( state == MODE_AUTONOMOUS )
        heartbeat_led_rate_.SetRate(HEARTBEAT_AUTO_HZ);
}

void HeartbeatLED::Update() {
    if ( heartbeat_led_rate_.NeedsRun() ) {
        digitalWrite(HEARTBEAT_LED, heartbeat_led_state_ ? HIGH : LOW);
        heartbeat_led_state_ = !heartbeat_led_state_;
    }
}
