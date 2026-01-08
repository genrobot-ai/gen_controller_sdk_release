#!/usr/bin/env bash
set -euo pipefail

# 用法示例：
#   单设备模式：
#     bash camera_cmd.sh 1234
#     bash camera_cmd.sh camerarc
#     bash camera_cmd.sh camerarl
#     bash camera_cmd.sh camerarr
#     bash camera_cmd.sh MCUID
#   双设备模式（左右区分）：
#     bash camera_cmd.sh left camerarc
#     bash camera_cmd.sh left camerarl
#     bash camera_cmd.sh left camerarr
#     bash camera_cmd.sh right camerarc
#     bash camera_cmd.sh right camerarl
#     bash camera_cmd.sh right camerarr
# 可选：通过环境变量 SERIAL_PORT 指定串口，默认为自动查找 /dev/ttyUSB*。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ✅ 关键修复：设置正确的PYTHONPATH
export PYTHONPATH="${SCRIPT_DIR}/lib/robot_driver:${PYTHONPATH:-}"

usage() {
  echo "用法:"
  echo "  单设备模式: bash ${BASH_SOURCE[0]} {1234|camerarc|camerarl|camerarr|MCUID}"
  echo "  双设备模式: bash ${BASH_SOURCE[0]} {left|right} {camerarc|camerarl|camerarr|MCUID|1234}"
  echo "可选环境变量: SERIAL_PORT=/dev/ttyUSB0 指定串口设备"
  exit 1
}

# 解析参数
if [[ $# -eq 1 ]]; then
  # 单设备模式
  SIDE=""
  RECORD_VALUE="$1"
elif [[ $# -eq 2 ]]; then
  # 双设备模式
  SIDE="$1"
  RECORD_VALUE="$2"
  if [[ "${SIDE}" != "left" && "${SIDE}" != "right" ]]; then
    echo "错误: 第一个参数必须是 'left' 或 'right'"
    usage
  fi
else
  usage
fi

# 校验RECORD_VALUE
case "${RECORD_VALUE}" in
  1234|camerarc|camerarl|camerarr|MCUID)
    ;;
  *)
    echo "错误: 第二个参数必须是 1234/camerarc/camerarl/camerarr/MCUID 之一"
    usage
    ;;
esac

# 支持通过环境变量指定串口，否则根据left/right自动选择
SERIAL_PORT="${SERIAL_PORT:-}"

# 如果没有指定串口，根据left/right设置默认设备
if [[ -z "${SERIAL_PORT}" ]]; then
  if [[ "${SIDE}" == "left" ]]; then
    SERIAL_PORT="/dev/ttyDeviceLeft"
  elif [[ "${SIDE}" == "right" ]]; then
    SERIAL_PORT="/dev/ttyDeviceRight"
  fi
  echo "使用默认设备: ${SERIAL_PORT:-自动查找}"
fi

# 根据传入的命令参数决定生成的YAML文件名
yaml_filename=""
if [[ "${RECORD_VALUE}" == "camerarc" ]]; then
  yaml_filename="cam0_sensor.yaml"
elif [[ "${RECORD_VALUE}" == "camerarl" ]]; then
  yaml_filename="cam1_sensor.yaml"
elif [[ "${RECORD_VALUE}" == "camerarr" ]]; then
  yaml_filename="cam2_sensor.yaml"
fi

# 如果指定了左右，添加前缀
if [[ -n "${SIDE}" && -n "${yaml_filename}" ]]; then
  yaml_filename="${SIDE}_${yaml_filename}"
fi

# ✅ 强制YAML文件生成在部署目录下
if [[ -n "${yaml_filename}" ]]; then
  CALIB_YAML_FILENAME="${SCRIPT_DIR}/${yaml_filename}"
  export CALIB_YAML_FILENAME
  echo "将生成YAML文件: ${CALIB_YAML_FILENAME}"
else
  unset CALIB_YAML_FILENAME
fi

echo "发送指令: ${RECORD_VALUE}, 设备: ${SIDE:-单设备}, 串口: ${SERIAL_PORT:-自动查找}"

# ✅ 修复版：直接运行databus_single.py的main函数
python3 - << PY
import sys
import os
import traceback
import time
import struct

# ✅ 关键修复：导入加密的databus_single.py
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, "lib/robot_driver"))

# 先导入databus_single.py，这会执行其解密和初始化
import databus_single

# 现在我们可以访问DataBus和find_serial_port
DataBus = getattr(databus_single, 'DataBus', None)
find_serial_port = getattr(databus_single, 'find_serial_port', None)

if not DataBus or not find_serial_port:
  # 如果直接导入失败，尝试从原始模块获取
  print("❌ 无法从databus_single.py获取DataBus或find_serial_port")
  print("可用的属性:", [a for a in dir(databus_single) if not a.startswith('_')])
  sys.exit(1)

record_value = "${RECORD_VALUE}"
serial_port_arg = "${SERIAL_PORT}" or None
side = "${SIDE}"

# 串口选择
serial_port = serial_port_arg
if not serial_port or serial_port == "None" or serial_port == "":
  # 传递side参数给find_serial_port
  serial_port = find_serial_port("ttyUSB", side=side)
  if not serial_port:
    print("❌ 未找到可用的串口设备")
    sys.exit(1)

print(f"✅ 使用串口: {serial_port}")
print(f"📤 发送相机标定指令: {record_value}")

# 创建回调函数来处理摄像头标定数据
def camera_calib_callback(camera_pack):
  """摄像头标定数据回调函数"""
  print("camera_pack received:", camera_pack)
  if camera_pack:
    print("✅ 摄像头标定数据解析成功")
  else:
    print("❌ 摄像头标定数据解析失败")

# ✅ 关键修复：创建DataBus实例时传递is_calib_cmd=True参数
# 这是最重要的修改，确保databus_single.py能正确解析标定数据
bus = DataBus(tty_port=serial_port, baudrate=921600, is_calib_cmd=True)
time.sleep(1.0)  # 预留设备初始化时间

# 注册回调
bus.register_camera_calib_callback(camera_calib_callback)

# ✅ 修复：从pack.py导入CmdPack
try:
  from pack import CmdPack
  print("✅ 成功导入CmdPack")
  
  # 发送命令
  record_bytes = record_value.encode('utf-8')
  cmd = CmdPack.pack_calib(record=record_bytes)
  bus.add_cmd(cmd)
  
except ImportError as e:
  print(f"❌ 无法导入CmdPack: {e}")
  # 尝试其他方法发送命令
  if hasattr(bus, 'send_camera_calib_cmd'):
    bus.send_camera_calib_cmd(record_value)
  else:
    print("❌ 无法发送命令")
    bus.stop()
    sys.exit(1)

# 等待设备响应（给予足够时间接收YAML数据）
print("等待设备响应...")
time.sleep(3.0)  # 延长等待时间确保YAML接收完成

# 特殊命令的处理
if record_value == "1234":
  print("Calibration OK !")
elif record_value == "MCUID":
  print("MCUID query executed")
else:
  print(f"完成发送 {record_value} 指令")

# 停止总线
bus.stop()
print("所有线程已停止")
PY
