#!/bin/bash
# Robot Driver 安装脚本

set -e

echo "========================================"
echo "    Robot Driver 安装程序"
echo "========================================"

if [ "$EUID" -ne 0 ]; then 
  echo "请使用sudo运行: sudo ./install.sh"
  exit 1
fi

INSTALL_DIR="/opt/robot_driver"
echo "安装到: $INSTALL_DIR"

# 备份
if [ -d "$INSTALL_DIR" ]; then
  BACKUP_DIR="$INSTALL_DIR.backup.$(date +%Y%m%d_%H%M%S)"
  echo "备份到: $BACKUP_DIR"
  mv "$INSTALL_DIR" "$BACKUP_DIR"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 创建目录
mkdir -p "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR/lib/robot_driver"
mkdir -p "$INSTALL_DIR/share/robot_driver/launch"
mkdir -p "$INSTALL_DIR/share/robot_driver/lib/robot_driver"

# 复制文件
echo "复制文件..."

# 1. 复制lib目录
cp -r "$SCRIPT_DIR/lib/robot_driver"/* "$INSTALL_DIR/lib/robot_driver/"

# 2. 复制launch文件
if [ -d "$SCRIPT_DIR/share/robot_driver/launch" ]; then
  cp -r "$SCRIPT_DIR/share/robot_driver/launch"/* "$INSTALL_DIR/share/robot_driver/launch/" 2>/dev/null || true
fi

# 3. 为rosrun复制文件
cp "$SCRIPT_DIR/lib/robot_driver/camera_view_single.py" "$INSTALL_DIR/share/robot_driver/lib/robot_driver/"
cp "$SCRIPT_DIR/lib/robot_driver/databus_single.py" "$INSTALL_DIR/share/robot_driver/lib/robot_driver/"
cp "$SCRIPT_DIR/lib/robot_driver/pack.py" "$INSTALL_DIR/share/robot_driver/lib/robot_driver/" 2>/dev/null || true
cp "$SCRIPT_DIR/lib/robot_driver/das_protocol.py" "$INSTALL_DIR/share/robot_driver/lib/robot_driver/" 2>/dev/null || true

# 4. 复制其他文件
cp "$SCRIPT_DIR/share/robot_driver/package.xml" "$INSTALL_DIR/share/robot_driver/"
cp "$SCRIPT_DIR/setup.bash" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/start_robot.sh" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/camera_cmd.sh" "$INSTALL_DIR/"

# 权限
chmod -R 755 "$INSTALL_DIR"
chown -R root:root "$INSTALL_DIR"
chmod +x "$INSTALL_DIR/lib/robot_driver"/*.py
chmod +x "$INSTALL_DIR/share/robot_driver/lib/robot_driver"/*.py
chmod +x "$INSTALL_DIR/setup.bash"
chmod +x "$INSTALL_DIR/start_robot.sh"
chmod +x "$INSTALL_DIR/camera_cmd.sh"

# 快捷命令
ln -sf "$INSTALL_DIR/start_robot.sh" /usr/local/bin/robot-driver
ln -sf "$INSTALL_DIR/lib/robot_driver/camera_view_single.py" /usr/local/bin/robot-camera
ln -sf "$INSTALL_DIR/lib/robot_driver/databus_single.py" /usr/local/bin/robot-das
ln -sf "$INSTALL_DIR/camera_cmd.sh" /usr/local/bin/camera-cmd

# udev规则
cat > /etc/udev/rules.d/99-robot-gripper.rules << 'UDEVRULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", MODE="0666"
UDEVRULE

udevadm control --reload-rules
udevadm trigger

echo "========================================"
echo "     ✅ 安装完成！"
echo "========================================"
echo ""
echo "使用方法:"
echo "  source /opt/robot_driver/setup.bash"
echo "  robot-driver"
echo ""
echo "摄像头标定命令:"
echo "  # 左设备"
echo "  camera-cmd left camerarc   # 生成 left_cam0_sensor.yaml"
echo "  camera-cmd left camerarl   # 生成 left_cam1_sensor.yaml"
echo "  camera-cmd left camerarr   # 生成 left_cam2_sensor.yaml"
echo "  camera-cmd left 1234       # 左设备标定完成"
echo "  camera-cmd left MCUID      # 查询左设备MCU ID"
echo ""
echo "  # 右设备"
echo "  camera-cmd right camerarc  # 生成 right_cam0_sensor.yaml"
echo "  camera-cmd right camerarl  # 生成 right_cam1_sensor.yaml"
echo "  camera-cmd right camerarr  # 生成 right_cam2_sensor.yaml"
echo "  camera-cmd right 1234      # 右设备标定完成"
echo "  camera-cmd right MCUID     # 查询右设备MCU ID"
