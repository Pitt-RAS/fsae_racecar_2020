#!/usr/bin/env bash

pushd $(dirname $BASH_SOURCE) > /dev/null

source robot.env

export ROS_MASTER_URI="http://${ROBOT_IP}:11311"
export ROS_IP=$(ip route get 8.8.8.8 | head -1 | cut -d' ' -f7)

popd > /dev/null
