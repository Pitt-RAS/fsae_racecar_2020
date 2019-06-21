#include "IMU.h"
#include "config.h"

IMU::IMU(ros::NodeHandle& imu_handle) :
        node_handle_(imu_handle),
        imu_publisher_("/platform/imu", &imu_msg_),
        imu_state_publisher_("/platform/imu_state", &imu_state_msg_),
        imu_() {
    imu_.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    // Setting IMU calibration data
    sensor_offsets_.accel_offset_x = ACCEL_X;
    sensor_offsets_.accel_offset_y = ACCEL_Y;
    sensor_offsets_.accel_offset_z = ACCEL_Z;
    sensor_offsets_.mag_offset_x = MAG_X;
    sensor_offsets_.mag_offset_y = MAG_Y;
    sensor_offsets_.mag_offset_z = MAG_Z;
    sensor_offsets_.gyro_offset_x = GYRO_X;
    sensor_offsets_.gyro_offset_y = GYRO_Y;
    sensor_offsets_.gyro_offset_z = GYRO_Z;
    sensor_offsets_.accel_radius = ACCEL_RADIUS;
    sensor_offsets_.mag_radius = MAG_RADIUS;
    //imu_.setSensorOffsets(sensor_offsets_);
    // ROS configuration
    imu_msg_.header.frame_id = "imu";
    node_handle_.advertise(imu_publisher_);
    node_handle_.advertise(imu_state_publisher_);
}

// Returns IMU heading
double IMU::GetHeading() {
    return imu_.getVector(Adafruit_BNO055::VECTOR_EULER).z();
}

// Publishes quaternion data and physical status of IMU
void IMU::Update() {
    uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
    imu_.getCalibration(&system, &gyro, &accel, &mag);
    // Set IMU State messages
    imu_state_msg_.system = system;
    imu_state_msg_.gyro = gyro;
    imu_state_msg_.accel = accel;
    imu_state_msg_.mag = mag;
    imu_state_publisher_.publish(&imu_state_msg_);
    // Checks if IMU is calibrate; if calibrated, publishes IMU data
    // Retrieve data from IMU
    imu::Quaternion quaternion = imu_.getQuat();
    imu::Vector<3> linear_accel = imu_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> angular_velocity = imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // Setting message data
    imu_msg_.header.stamp = node_handle_.now();
    imu_msg_.orientation.x = quaternion.x();
    imu_msg_.orientation.y = quaternion.y();
    imu_msg_.orientation.z = quaternion.z();
    imu_msg_.orientation.w = quaternion.w();
    imu_msg_.linear_acceleration.x = linear_accel.x();
    imu_msg_.linear_acceleration.y = linear_accel.y();
    imu_msg_.linear_acceleration.z = linear_accel.z();
    imu_msg_.angular_velocity.x = angular_velocity.x() * (M_PI / 180.0);
    imu_msg_.angular_velocity.y = angular_velocity.y() * (M_PI / 180.0);
    imu_msg_.angular_velocity.z = angular_velocity.z() * (M_PI / 180.0);

    imu_msg_.orientation_covariance[8] = kIMUOrientationVariance;

    imu_msg_.linear_acceleration_covariance[0] = kIMUAccelVariance[0];
    imu_msg_.linear_acceleration_covariance[4] = kIMUAccelVariance[1];

    imu_msg_.angular_velocity_covariance[8] = kIMUGyroVariance;

    imu_publisher_.publish(&imu_msg_);
}
