#!/usr/bin/env bash
chmod 744 launch/launch-wrapper.sh
./launch/launch-wrapper.sh roscore
./launch/launch-wrapper.sh rosrun xbox_teleop xbox_teleop
./launch/launch-wrapper.sh rosrun arduino_node arduino
