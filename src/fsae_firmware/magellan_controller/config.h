#ifndef CONFIG_H
#define CONFIG_H

#include "RobotState.h"
#include "math.h"

extern RobotState currentState;

#define HEARTBEAT_LED 13
#define HEARTBEAT_DISABLED_HZ 1
#define HEARTBEAT_AUTO_HZ 10
#define HEARTBEAT_TELEOP_HZ 5

// R9 transmitter
// Timeout before we consider the transmitter disconnected
#define TX_TIMEOUT 2000
#define TX_SERIALPORT Serial1

// Loop rates
#define MAIN_LOOP_HZ 100
#define XBEE_LOOP_HZ 3
#define ENCODER_UPDATE_HZ 60
#define DEBUG_HZ 5

#define ESC_PWM 3 // Throttle
#define SERVO_PWM 4

// Steering trim
#define STEERING_OFFSET 0

#define THROTTLE_MIN 0.1
#define STEERING_MIN 0.05

#define ENCODER_A 6
#define ENCODER_B 7
#define ENCODER_C 8

#define RUN_BUTTON 21



// Distance from front wheel to back wheel
const constexpr double kTrackLength = 0.3175;
// Distance from left wheel to right wheel
const constexpr double kTrackWidth = 0.24;
// Max velocity
const constexpr double kMaxVelocity = 3.0;

// Max turning angle of the inside wheel in a turn
const constexpr double kMaxTurningAngle = M_PI / 6.0;

const constexpr double kIMUAccelVariance[2] = {10, 10};
const constexpr double kIMUOrientationVariance = 1e-5;
const constexpr double kIMUGyroVariance = 0.00001;
const constexpr double kVelocityVariance = 1;
const constexpr double kDistancePerTick = (0.079 * 3.14) / 10.0;

#endif
