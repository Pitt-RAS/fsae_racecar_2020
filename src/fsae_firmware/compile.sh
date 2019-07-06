#!/usr/bin/env bash

if [ -z ${ARDUINO_PATH} ];
then
    echo "Set ARDUINO_PATH to point to your Arduino install."
    exit 1
fi

pushd $(dirname $0) > /dev/null

set -e

# Rebuild ROS Arduino libraries
rm -rf ${HOME}/Arduino/libraries/ros_lib
mkdir -p ${HOME}/Arduino/libraries
rosrun rosserial_arduino make_libraries.py ${HOME}/Arduino/libraries/

mkdir -p /tmp/fsae-build
${ARDUINO_PATH}/arduino-builder \
    -compile \
    -build-path /tmp/fsae-build \
    -logger=machine \
    -hardware ${ARDUINO_PATH}/hardware \
    -tools ${ARDUINO_PATH}/tools-builder \
    -tools ${ARDUINO_PATH}/hardware/tools/avr \
    -built-in-libraries ${ARDUINO_PATH}/libraries \
    -libraries ~/Arduino/libraries \
    -fqbn=teensy:avr:teensy31:usb=serial,speed=96,opt=o2std,keys=en-us \
    -verbose \
    firmware/magellan_controller/magellan_controller.ino

popd > /dev/null
