# USB Configuration Guide

This guide explains how to configure udev rules so each Gen Controller USB port maps to stable serial and camera device names.

The template file is located at [config/99-usb-serial.rules](../config/99-usb-serial.rules).

After configuration, the corresponding USB port can recognize Gen Controller devices without further setup.

[中文](usb-setup_CN.md)

## 1 Single Gripper USB Port

The final configuration should look like the figure below.

![Single gripper udev example](../image/image_1.png)

Fields that must be modified:

![Fields to modify](../image/image_2.png)

### 1.1 Parameter 1 — Serial Port KERNELS

```shell
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB* | grep -E "KERNELS|DRIVERS"
```

If the serial port cannot be detected:

```shell
sudo apt remove brltty
```

Configure the **second** `KERNELS` value from the output to Parameter 1:

![Serial KERNELS example](../image/image_3.png)

### 1.2 Parameter 2 — Camera KERNELS

```shell
v4l2-ctl --list-devices
```

Example output:

![v4l2 device list](../image/image_4.png)

For the first camera of this USB device, run:

```shell
udevadm info -a -n /dev/video* | grep -E "KERNELS|SUBSYSTEMS"
```

Configure the **first** `KERNELS` value from the output to Parameter 2:

![Camera KERNELS example](../image/image_5.png)

### 1.3 Apply Rules

```shell
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 2 Dual Gripper USB Ports

The final configuration should look like the figure below.

![Dual gripper udev example](../image/image_6.png)

Fields to modify:

![Dual gripper fields](../image/image_7.png)

Steps:

1. Plug in the **left** gripper and configure it using the single gripper method above.
2. Unplug the left gripper, plug in the **right** gripper, and configure it the same way.
3. Reload udev rules.

Default serial symlinks after dual configuration:

| Device | Symlink |
|--------|---------|
| Left gripper | `/dev/ttyDeviceLeft` |
| Right gripper | `/dev/ttyDeviceRight` |

## 3 Multiple Grippers

Add additional entries to `99-usb-serial.rules` for each gripper, following the same pattern.

## 4 Verify Device ID (Optional)

With `roscore` running:

```shell
cd src/robot_driver/scripts/
bash camera_cmd.sh MCUID
```

Example output:

![Device ID example](../image/image_8.jpg)
