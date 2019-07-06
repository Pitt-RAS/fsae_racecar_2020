#ifndef ROBOT_H
#define ROBOT_H

#include "RobotState.h"
#include "TransmitterInterface.h"
#include "Rate.h"
#include "config.h"
#include <ros.h>
#include "HeartbeatLED.h"
#include "PWM.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "IMU.h"
#include "EncoderPublisher.h"
#include "Drivetrain.h"

class Robot {
public:
    Robot(ros::NodeHandle& nh);

    void TeleopInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
    void DisabledInit();
    void DisabledPeriodic();
    void AlwaysPeriodic();
    void UpdateThrottle(const std_msgs::Float64& cmd_throttle_percent_);
    void UpdateSteering(const std_msgs::Float64& cmd_steering_angle_);

    void Update();
private:
    RobotState current_state_;
    ros::NodeHandle& nh_;
    TransmitterInterface transmitter_;
    HeartbeatLED heartbeat_;
    ros::Subscriber<std_msgs::Float64, Robot> throttle_subscriber_;
    float throttle_percent_;
    ros::Subscriber<std_msgs::Float64, Robot> steering_subscriber_;
    float steering_angle_;
    std_msgs::Int32 user_input_msg_;
    ros::Publisher user_input_publisher;
    Rate user_input_msg_rate_;
    IMU imu_;
    EncoderPublisher encoder_publisher_;
    Drivetrain drivetrain_;
};

#endif
