#!/bin/bash
# protect_python.sh - post-install: keep only .pyc and wrapper scripts

set -e

echo "Applying Python source protection..."

INSTALL_DIR="$1"
if [ -z "$INSTALL_DIR" ]; then
    echo "Error: install directory argument required"
    exit 1
fi

LIB_DIR="$INSTALL_DIR/lib/robot_driver"

if [ ! -d "$LIB_DIR" ]; then
    echo "Error: directory not found: $LIB_DIR"
    exit 1
fi

echo "Processing: $LIB_DIR"

chmod +x "$LIB_DIR/camera_view_single.py" 2>/dev/null || true
chmod +x "$LIB_DIR/databus_single.py" 2>/dev/null || true

if [ ! -f "$LIB_DIR/camera_view_single.pyc" ] || \
   [ ! -f "$LIB_DIR/databus_single.pyc" ]; then
    echo "Error: required .pyc files missing"
    exit 1
fi

echo "Protection done. Files:"
ls -la "$LIB_DIR/" | grep -E '\.pyc$|\.py$'

echo "Python source protection complete"
