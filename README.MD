# Gen Gripper Controller SDK

> ROS1 driver for Gen Controller gripper — camera streaming, tactile sensing, encoder feedback, and distance control.

[中文](README(ZH).MD)

[GitHub repository](https://github.com/genrobot-ai/gen_controller_sdk_release)

License: [MIT License](LICENSE.txt)

## 1 Features

- Three-camera image streaming with optional live preview
- Tactile sensor data publishing (left / right)
- Encoder feedback for gripper opening distance
- Gripper distance control via ROS topics
- Single-gripper and dual-gripper launch files
- Utility scripts for calibration, device ID, and tactile debugging

## 2 Requirements


| Item     | Requirement                   |
| -------- | ----------------------------- |
| OS       | Ubuntu 20.04 (recommended)    |
| ROS      | ROS1 Noetic                   |
| Python   | 3.8+                          |
| USB      | USB 3.0 port                  |
| Hardware | Gen Controller gripper device |


## Important Notes

Before operating the gripper, observe the following:

1. **Initial Calibration**: On first command, ensure the gripper is unobstructed (calibration code: -66.66).
2. **Power Supply**: Error code -6.6 indicates unpowered motor. Connect 24V power to the XT30 connector.
3. **Power Cycle Recalibration**: After power loss and restart, recalibration is required. Remove any objects from the gripper before power restoration.
4. **Power Cycle Delay**: Wait at least 3 seconds after power-off before powering on the motor again. Avoid rapid power cycling to prevent motor damage.


## 3 Quick Start

> First-time users must complete [USB configuration](docs/usb-setup.md) before launching the driver.

```shell
git clone https://github.com/genrobot-ai/gen_controller_sdk_release.git
cd gen_controller_sdk_release
catkin_make
source devel/setup.bash
roslaunch robot_driver single_gripper_start.launch
```

Verify feedback and send a control command:

```shell
# Terminal 2 — read encoder feedback
rostopic echo /encoder

# Terminal 3 — set target opening to 5 cm
rostopic pub /target_distance std_msgs/Float32 "data: 0.05"
```

Select the gripper type in the launch command. The maximum opening distance depends on the gripper:

| Gripper type | Maximum opening distance |
| --- | ---: |
| `default_gripper` | `0.103` m |
| `tactile_gripper` | `0.0998` m |
| `soft_gripper` | `0.103` m |



## 4 ROS Topics



### 4.1 Single Gripper


| Topic                       | Type                      | Direction | Description                                   |
| --------------------------- | ------------------------- | --------- | --------------------------------------------- |
| `/camera/color/image_raw`   | `sensor_msgs/Image`       | publish   | Center camera                                 |
| `/camera_1/color/image_raw` | `sensor_msgs/Image`       | publish   | Left-side camera                              |
| `/camera_2/color/image_raw` | `sensor_msgs/Image`       | publish   | Right-side camera                             |
| `/encoder`                  | `std_msgs/Float32`        | publish   | Gripper opening distance feedback (m)         |
| `/tactile/left`             | `std_msgs/Int8MultiArray` | publish   | Left tactile sensor                           |
| `/tactile/right`            | `std_msgs/Int8MultiArray` | publish   | Right tactile sensor                          |
| `/target_distance`          | `std_msgs/Float32`        | subscribe | Target opening distance; keep within the selected gripper type limit |




### 4.2 Dual Gripper

All topics are prefixed with `/left_gripper` or `/right_gripper`. For example:


| Topic                            | Type               | Direction | Description                    |
| -------------------------------- | ------------------ | --------- | ------------------------------ |
| `/left_gripper/encoder`          | `std_msgs/Float32` | publish   | Left gripper opening feedback  |
| `/left_gripper/target_distance`  | `std_msgs/Float32` | subscribe | Left gripper target distance   |
| `/right_gripper/encoder`         | `std_msgs/Float32` | publish   | Right gripper opening feedback |
| `/right_gripper/target_distance` | `std_msgs/Float32` | subscribe | Right gripper target distance  |


Camera and tactile topics follow the same namespace pattern (`/left_gripper/camera/...`, `/left_gripper/tactile/...`, etc.).

## 5 Installation



### 5.1 Install system and Python dependencies

```shell
sudo apt update
sudo apt install ros-noetic-desktop-full python3-pip v4l-utils
pip3 install -r requirements.txt
```

`v4l-utils` provides `v4l2-ctl`, required for USB configuration.

### 5.2 Clone and build

```shell
git clone https://github.com/genrobot-ai/gen_controller_sdk_release.git
cd gen_controller_sdk_release
catkin_make
source devel/setup.bash
```

To load the workspace automatically, add to `~/.bashrc`:

```shell
source /path/to/gen_controller_sdk_release/devel/setup.bash
```



## 6 USB Configuration

Configure udev rules once per USB port before first use. The template is at [config/99-usb-serial.rules](./config/99-usb-serial.rules).

Summary:

1. Query serial and camera `KERNELS` values with `udevadm` and `v4l2-ctl`
2. Edit `config/99-usb-serial.rules`
3. Copy to `/etc/udev/rules.d/` and reload rules

For step-by-step instructions with screenshots, see:

- [USB 配置指南 (ZH)](docs/usb-setup_CN.md)
- [USB Configuration Guide (EN)](docs/usb-setup.md)

Default serial symlinks after dual-gripper setup: `/dev/ttyDeviceLeft`, `/dev/ttyDeviceRight`.

## 7 Usage



### 7.1 Single Gripper

```shell
source devel/setup.bash
roslaunch robot_driver single_gripper_start.launch
roslaunch robot_driver single_gripper_start.launch gripper_type:=tactile_gripper
roslaunch robot_driver single_gripper_start.launch gripper_type:=soft_gripper
```

Three preview windows appear by default. Disable preview:

```shell
roslaunch robot_driver single_gripper_start.launch show_preview:=false
```



### 7.2 Dual Gripper

```shell
source devel/setup.bash
roslaunch robot_driver dual_gripper_start.launch
roslaunch robot_driver dual_gripper_start.launch left_gripper_type:=tactile_gripper right_gripper_type:=default_gripper
```

Run demo scripts to bridge model commands to gripper topics:

```shell
cd src/robot_driver/scripts/
python3 left_das_controller_infer.py
python3 right_das_controller_infer.py
```



### 7.3 Device Utilities

These commands require a running `roscore`. Do **not** run multiple device commands simultaneously.

**Single device:**

```shell
roscore
cd src/robot_driver/scripts/
bash camera_cmd.sh camerarc   # Center camera calibration
bash camera_cmd.sh camerarl   # Left camera calibration
bash camera_cmd.sh camerarr   # Right camera calibration
bash camera_cmd.sh MCUID      # Device ID
python3 tactile_dual_print.py # Print tactile data
```

**Dual device (left / right):**

```shell
roscore
cd src/robot_driver/scripts/

bash camera_cmd.sh left camerarc
bash camera_cmd.sh left camerarl
bash camera_cmd.sh left camerarr
bash camera_cmd.sh left camerar2c
bash camera_cmd.sh left camerar2l
bash camera_cmd.sh left camerar2r
bash camera_cmd.sh left MCUID
bash camera_cmd.sh left VERSION
python3 tactile_dual_print.py _gripper_ns:=left_gripper

bash camera_cmd.sh right camerarc
bash camera_cmd.sh right camerarl
bash camera_cmd.sh right camerarr
bash camera_cmd.sh right camerar2c
bash camera_cmd.sh right camerar2l
bash camera_cmd.sh right camerar2r
bash camera_cmd.sh right MCUID
bash camera_cmd.sh right VERSION
python3 tactile_dual_print.py _gripper_ns:=right_gripper
```

Notes:
- `camerarl`, `camerarc`, `camerarr` are for `640x480` calibration files.
- `camerar2l`, `camerar2c`, `camerar2r` are for `1600x1296` calibration files.
- `VERSION` can query the current software version. If no response is returned, the device firmware/software is too old.



## 8 Troubleshooting


| Problem                         | Solution                                                           |
| ------------------------------- | ------------------------------------------------------------------ |
| Serial port not found           | Run `sudo apt remove brltty`, then replug the device               |
| Camera or serial has wrong path | Re-check udev rules; see [docs/usb-setup.md](docs/usb-setup.md)    |
| V4 Controller frame rate halved | Set `<param name="fps" value="60" />` in the launch file           |
| No ROS data                     | Ensure `roscore` is running and udev rules are loaded              |
| `catkin_make` fails             | Confirm ROS Noetic is sourced: `source /opt/ros/noetic/setup.bash` |




## 9 Documentation


| Description           | Link                                                                                     |
| --------------------- | ---------------------------------------------------------------------------------------- |
| User Manual           | [DAS Controller User Manual](https://docs.genrobot.ai/en/products/das-controller)        |
| URDF                  | [URDF Model](https://zcnma1sv5kma.feishu.cn/wiki/CKpbwye45iOlrckukIPc5hKCndh)            |
| Troubleshooting Guide | [Problem Solving Guide](https://zcnma1sv5kma.feishu.cn/docx/E6VJdNqIfoGrSDxKIHKck7tLn3b) |
| USB setup (ZH)           | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)                                             |
| USB setup (EN)        | [docs/usb-setup.md](docs/usb-setup.md)                                                   |
| udev rules template   | [config/99-usb-serial.rules](config/99-usb-serial.rules)                                 |
| Single gripper launch | [single_gripper_start.launch](src/robot_driver/launch/single_gripper_start.launch)       |
| Dual gripper launch   | [dual_gripper_start.launch](src/robot_driver/launch/dual_gripper_start.launch)           |
| Demo scripts          | [src/robot_driver/scripts/](src/robot_driver/scripts/)                                   |

