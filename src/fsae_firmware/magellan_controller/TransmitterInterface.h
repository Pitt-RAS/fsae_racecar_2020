#ifndef TRANSMITTER_INTERFACE_H
#define TRANSMITTER_INTERFACE_H

#include <ros.h>
#include <SBUS.h>
#include "SoftWatchdog.h"

class TransmitterInterface {
public:
    TransmitterInterface(ros::NodeHandle& nh);

    double throttle_percent();
    double steering_angle();
    uint16_t user_setting();

    bool WantsEnable();
    bool WantsAutonomous();
    bool IsConnected();

    void Update();
private:
    ros::NodeHandle& nh_;
    SBUS r9_;
    SoftWatchdog watchdog_;
    uint16_t channels_[16];
    bool fail_safe_;
    bool lost_frame_;

    double throttle_percent_;
    double steering_angle_;
    bool enabled_;
    bool autonomous_;
    uint16_t user_;
};

#endif
