#include "config.h"
#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface(ros::NodeHandle& nh) :
        nh_(nh),
        r9_(TX_SERIALPORT),
        watchdog_(TX_TIMEOUT),
        throttle_percent_(0),
        steering_angle_(0),
        enabled_(false),
        autonomous_(false),
        user_(0) {
    r9_.begin();
}

void TransmitterInterface::Update() {
    if ( r9_.read(&channels_[0], &fail_safe_, &lost_frame_) ) {
        enabled_ = channels_[4] > 1500;
        autonomous_ = channels_[7] > 1500;
        user_ = channels_[6];

        // Fail safe is only set if the radio disconnects
        // This watchdog handles if the receiver disconnects
        watchdog_.Feed();
  
        throttle_percent_ = channels_[0] - 172;
        // Scale 0 to positive 1
        throttle_percent_ /= 1640;

        // Center around 0
        throttle_percent_ -= 0.5;
        throttle_percent_ *= 2;

        steering_angle_ = channels_[1] - 1000;
        // Scale -90 to +90
        steering_angle_ /= 828;
        steering_angle_ *= -90;
    }
}

bool TransmitterInterface::WantsEnable() {
    return enabled_ && IsConnected();
}

bool TransmitterInterface::WantsAutonomous() {
    return autonomous_ && IsConnected();
}

bool TransmitterInterface::IsConnected() {
    return !watchdog_.Hungry() && !fail_safe_;
}

double TransmitterInterface::throttle_percent() {
    return throttle_percent_;
}

double TransmitterInterface::steering_angle() {
    return steering_angle_;
}

uint16_t TransmitterInterface::user_setting() {
    return user_;
}
