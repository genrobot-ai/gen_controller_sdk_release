#!/bin/bash
# Robot Driver 环境设置

_ROBOT_DRIVER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PYTHONPATH="$_ROBOT_DRIVER_ROOT/lib:$PYTHONPATH"
export ROS_PACKAGE_PATH="$_ROBOT_DRIVER_ROOT/share:$ROS_PACKAGE_PATH"

echo "✅ Robot Driver 环境已设置"
