#!/bin/bash
# Robot Driver 启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup.bash"

export ROS_PACKAGE_PATH="$SCRIPT_DIR/share:$ROS_PACKAGE_PATH"

# 查找launch文件
LAUNCH_DIR="$SCRIPT_DIR/share/robot_driver/launch"
if [ ! -d "$LAUNCH_DIR" ]; then
  echo "错误: 找不到launch目录"
  exit 1
fi

LAUNCH_FILES=("$LAUNCH_DIR"/*.launch)
if [ ${#LAUNCH_FILES[@]} -eq 0 ] || [ ! -f "${LAUNCH_FILES[0]}" ]; then
  echo "错误: 找不到launch文件"
  exit 1
fi

LAUNCH_FILE="${LAUNCH_FILES[0]}"
echo "启动: $(basename "$LAUNCH_FILE")"

if [ -z "$ROS_MASTER_URI" ]; then
  export ROS_MASTER_URI=http://localhost:11311
fi

roslaunch robot_driver "$(basename "$LAUNCH_FILE")" "$@"
