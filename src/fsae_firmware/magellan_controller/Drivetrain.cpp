#include "Drivetrain.h"
#include "config.h"
#include <cmath>

Drivetrain::Drivetrain(ros::NodeHandle& nh) :
        last_commanded_percent_(0),
        nh_(nh),
        throttle_pwm_(ESC_PWM),
        steering_pwm_(SERVO_PWM),
        steering_angle_msg_(),
        steering_angle_publisher_("/platform/turning_radius", &steering_angle_msg_) {
    throttle_pwm_.ConfigLowLimit(THROTTLE_MIN);
    steering_pwm_.ConfigOffset(STEERING_OFFSET);
    steering_pwm_.ConfigLowLimit(STEERING_MIN);

    nh.advertise(steering_angle_publisher_);
}

void Drivetrain::SetThrottlePercent(double percent) {
    throttle_pwm_.Set(percent);

    if ( fabs(percent) > 0.1 )
        last_commanded_percent_ = percent;
}

void Drivetrain::SetSteeringPercent(double percent) {
    percent = -percent;
    steering_pwm_.Set(percent);
    steering_angle_msg_.data = GetTurningRadius(percent);
    steering_angle_publisher_.publish(&steering_angle_msg_);
}

void Drivetrain::SetSteeringAngle(double angle) {
    SetSteeringPercent(GetPercentForSteeringAngle(angle));
}

double Drivetrain::GetSteeringAngleForPercent(double percent) {
    return percent * kMaxTurningAngle;
}

double Drivetrain::GetPercentForSteeringAngle(double angle) {
    if ( std::abs(angle) > kMaxTurningAngle )
        return std::copysign(1, angle);
    return angle / kMaxTurningAngle;
}

// Returns radius of imaginary circle the car is driving along while turning
double Drivetrain::GetTurningRadius(double percent) {
    if ( percent == 0 )
        return 0;

    double angle = GetSteeringAngleForPercent(percent);
    return kTrackLength * (1.0 / tan(angle)) + (kTrackWidth / 2.0);
}

double Drivetrain::GetSteeringAngle(double radius) {
    return std::copysign(atan(kTrackLength / (std::abs(radius) + (kTrackWidth / 2))), radius);
}

bool Drivetrain::DirectionIsForward() {
    return last_commanded_percent_ >= 0;
}
