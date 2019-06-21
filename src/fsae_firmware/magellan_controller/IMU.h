#ifndef IMU_H_
#define IMU_H_

#include <ros.h>
#include <Adafruit_BNO055.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <magellan_core/IMUState.h>

class IMU {
public:
    IMU(ros::NodeHandle& imu_handle);
    double GetHeading();
    void Update();
private:
    ros::NodeHandle& node_handle_;
    ros::Publisher imu_publisher_;
    ros::Publisher imu_state_publisher_;
    Adafruit_BNO055 imu_;
    sensor_msgs::Imu imu_msg_;
    magellan_core::IMUState imu_state_msg_;
    adafruit_bno055_offsets_t sensor_offsets_;
    // IMU constants
    const int16_t ACCEL_X = -7;
    const int16_t ACCEL_Y = -30;
    const int16_t ACCEL_Z = 24;
    const int16_t MAG_X = 35;
    const int16_t MAG_Y = -112;
    const int16_t MAG_Z = -536;
    const int16_t GYRO_X = -2;
    const int16_t GYRO_Y = 0;
    const int16_t GYRO_Z = -1;
    const int16_t ACCEL_RADIUS = 1000;
    const int16_t MAG_RADIUS = 896;
};

#endif
