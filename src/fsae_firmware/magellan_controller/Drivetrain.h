#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "config.h"
#include <ros.h>
#include <std_msgs/Float64.h>
#include "PWM.h"

class Drivetrain {
public:
    Drivetrain(ros::NodeHandle& nh);

    void SetThrottlePercent(double percent);
    void SetSteeringPercent(double percent);
    void SetSteeringAngle(double angle);
    bool DirectionIsForward();

    double GetSteeringAngleForPercent(double percent);
    double GetPercentForSteeringAngle(double angle);
    double GetTurningRadius(double percent);
    double GetSteeringAngle(double radius);
private:
    double last_commanded_percent_;
    ros::NodeHandle& nh_;
    PWM throttle_pwm_;
    PWM steering_pwm_;

    std_msgs::Float64 steering_angle_msg_;
    ros::Publisher steering_angle_publisher_;
};

#endif
