#!/bin/bash

echo "Starting robot system..."
# source ~/robot_ws/devel/setup.bash

if [ "$1" = "simple" ]; then
    echo "Starting in simple mode..."
    roslaunch robot_driver single_gripper_start.launch
elif [ "$1" = "preview" ]; then
    echo "Starting with preview..."
    roslaunch robot_driver config_gripper_start.launch show_preview:=true
elif [ "$1" = "serial" ] && [ -n "$2" ]; then
    echo "Starting with serial port: $2"
    roslaunch robot_driver config_gripper_start.launch serial_port:=$2
else
    echo "Default launch..."
    echo "Options:"
    echo "  simple    - simple mode (no preview)"
    echo "  preview   - preview mode (camera windows)"
    echo "  serial /dev/ttyUSB0 - explicit serial device"
    roslaunch robot_driver config_gripper_start.launch
fi
