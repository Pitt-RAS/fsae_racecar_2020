#include "Robot.h"

Robot::Robot(ros::NodeHandle& nh) :
        current_state_(MODE_DISABLED),
        nh_(nh),
        transmitter_(nh),
        heartbeat_(),
        throttle_subscriber_("/platform/cmd_velocity", &Robot::UpdateThrottle, this),
        throttle_percent_(0.0),
        steering_subscriber_("/platform/cmd_turning_radius", &Robot::UpdateSteering, this),
        steering_angle_(0.0),
        user_input_msg_(),
        user_input_publisher("/platform/user_input", &user_input_msg_),
        user_input_msg_rate_(1),
        imu_(nh),
        encoder_publisher_(nh),
        drivetrain_(nh) {
    nh.subscribe(throttle_subscriber_);
    nh.subscribe(steering_subscriber_);
    nh.advertise(user_input_publisher);

    DisabledInit();
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
    // Get the throttle percent from the Transmitter Interface and pass it to PWM
    drivetrain_.SetThrottlePercent(transmitter_.throttle_percent());

    // Get the steering angle from the Transmitter Interface and pass it to PWM
    drivetrain_.SetSteeringPercent(transmitter_.steering_angle() / 90.0);
}

void Robot::AutonomousInit() {
    drivetrain_.SetThrottlePercent(0);
    drivetrain_.SetSteeringPercent(0);

    throttle_percent_ = 0.0;
    steering_angle_ = 0.0;
}

void Robot::AutonomousPeriodic() {
    drivetrain_.SetThrottlePercent(throttle_percent_);
    drivetrain_.SetSteeringAngle(-steering_angle_);
}

void Robot::DisabledInit() {
    drivetrain_.SetThrottlePercent(0);
    drivetrain_.SetSteeringPercent(0);
}

void Robot::DisabledPeriodic() {
}

void Robot::AlwaysPeriodic() {
    encoder_publisher_.Update(drivetrain_.DirectionIsForward());
    imu_.Update();
}

void Robot::UpdateThrottle(const std_msgs::Float64& cmd_velocity) {
    throttle_percent_ = cmd_velocity.data / kMaxVelocity;
}

void Robot::UpdateSteering(const std_msgs::Float64& cmd_turning_radius) {
    steering_angle_ = drivetrain_.GetSteeringAngle(cmd_turning_radius.data);
}

void Robot::Update() {
    // Subsystem updates
    transmitter_.Update();
    heartbeat_.Update();

    if ( transmitter_.WantsEnable() && transmitter_.WantsAutonomous() && current_state_ != MODE_AUTONOMOUS ) {
        AutonomousInit();
        current_state_ = MODE_AUTONOMOUS;
        heartbeat_.SetState(current_state_);
    }
    else if ( transmitter_.WantsEnable() && !transmitter_.WantsAutonomous() && current_state_ != MODE_TELEOP ) {
        TeleopInit();
        current_state_ = MODE_TELEOP;
        heartbeat_.SetState(current_state_);
    }
    else if ( !transmitter_.WantsEnable() && current_state_ != MODE_DISABLED ) {
        DisabledInit();
        current_state_ = MODE_DISABLED;
        heartbeat_.SetState(current_state_);
    }

    if ( transmitter_.WantsEnable() ) {
        user_input_msg_.data = transmitter_.user_setting();
        user_input_publisher.publish(&user_input_msg_);
    }

    if ( current_state_ == MODE_DISABLED )
        DisabledPeriodic();
    else if ( current_state_ == MODE_AUTONOMOUS )
        AutonomousPeriodic();
    else if ( current_state_ == MODE_TELEOP )
        TeleopPeriodic();

    AlwaysPeriodic();
}
