#!/usr/bin/env bash
set -euo pipefail

# Usage examples:
#   Single device:
#     bash camera_cmd.sh 1234
#     bash camera_cmd.sh camerarc
#     bash camera_cmd.sh camerarl
#     bash camera_cmd.sh camerarr
#     bash camera_cmd.sh MCUID
#   Dual device (left/right):
#     bash camera_cmd.sh left camerarc
#     bash camera_cmd.sh right camerarl
#     bash camera_cmd.sh right camerarr
# Optional: set SERIAL_PORT to force a device; otherwise auto-find /dev/ttyUSB*.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

usage() {
  echo "Usage:"
  echo "  Single: bash ${BASH_SOURCE[0]} {1234|camerarc|camerarl|camerarr|MCUID}"
  echo "  Dual:   bash ${BASH_SOURCE[0]} {left|right} {camerarc|camerarl|camerarr|MCUID|1234}"
  echo "Optional env: SERIAL_PORT=/dev/ttyUSB0"
  exit 1
}

# Parse args
if [[ $# -eq 1 ]]; then
  SIDE=""
  RECORD_VALUE="$1"
elif [[ $# -eq 2 ]]; then
  SIDE="$1"
  RECORD_VALUE="$2"
  if [[ "${SIDE}" != "left" && "${SIDE}" != "right" ]]; then
    echo "Error: first argument must be 'left' or 'right'"
    usage
  fi
else
  usage
fi

# Validate RECORD_VALUE
case "${RECORD_VALUE}" in
  1234|camerarc|camerarl|camerarr|MCUID)
    ;;
  *)
    echo "Error: second argument must be one of 1234/camerarc/camerarl/camerarr/MCUID"
    usage
    ;;
esac

SERIAL_PORT="${SERIAL_PORT:-}"

# Default mapped devices when SERIAL_PORT unset
if [[ -z "${SERIAL_PORT}" ]]; then
  if [[ "${SIDE}" == "left" ]]; then
    SERIAL_PORT="/dev/ttyDeviceLeft"
  elif [[ "${SIDE}" == "right" ]]; then
    SERIAL_PORT="/dev/ttyDeviceRight"
  fi
  echo "Using default device: ${SERIAL_PORT:-auto-detect}"
fi

# YAML output filename from command
yaml_filename=""
if [[ "${RECORD_VALUE}" == "camerarc" ]]; then
  yaml_filename="cam0_sensor.yaml"
elif [[ "${RECORD_VALUE}" == "camerarl" ]]; then
  yaml_filename="cam1_sensor.yaml"
elif [[ "${RECORD_VALUE}" == "camerarr" ]]; then
  yaml_filename="cam2_sensor.yaml"
fi

if [[ -n "${SIDE}" && -n "${yaml_filename}" ]]; then
  yaml_filename="${SIDE}_${yaml_filename}"
fi

if [[ -n "${yaml_filename}" ]]; then
  export CALIB_YAML_FILENAME="${yaml_filename}"
  echo "Will write YAML: ${yaml_filename}"
else
  unset CALIB_YAML_FILENAME
fi

echo "Command: ${RECORD_VALUE}, device: ${SIDE:-single}, serial: ${SERIAL_PORT:-auto-detect}"

python3 - "$RECORD_VALUE" "$SERIAL_PORT" <<'PY'
import sys
import time
import os

record_value = sys.argv[1]
serial_port_arg = sys.argv[2] or None

from databus_single import DataBus, find_serial_port  # noqa: E402
from pack import CmdPack  # noqa: E402

record_bytes = record_value.encode("utf-8")

serial_port = serial_port_arg
if not serial_port or serial_port == "None" or serial_port == "":
    serial_port = find_serial_port("ttyUSB")

if not serial_port:
    print("No serial port found")
    sys.exit(1)

print(f"Using serial: {serial_port}")
print(f"Sending camera calib command: {record_value}")

# MCUID prints payload between das\\r\\n
bus = DataBus(tty_port=serial_port, baudrate=921600, is_calib_cmd=True, calib_cmd_name=record_value)
time.sleep(1.0)
bus.add_cmd(CmdPack.pack_calib(record=record_bytes))
time.sleep(0.5)
bus.stop()

if record_value == "1234":
    print("Calibration OK !")
elif record_value == "MCUID":
    print("MCUID query executed")
else:
    print(f"Finished sending command: {record_value}")

PY
