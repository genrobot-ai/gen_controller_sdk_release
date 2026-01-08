# genrobot_controller_sdk
## Environment Deployment
```
PC System Requirements: Ubuntu 20.04
Communication: ROS1 System
Configure environment according to requirements.txt
USB port must be version 3.0
```

## Enter Directory
```
cd genrobot_controller_sdk
```

## Configure USB Interface

### Single Gripper USB Port Configuration
The final configuration is as shown in the figure. After configuration, this USB port can recognize any Gen Controller, and no further configuration is required. The template file is stored in 
```
config/99-usb-serial.rules
```
![image/image_1.png](image/image_1.png)  

The part that users need to modify is:    
![image/image_2.png](image/image_2.png)  

Modification method for Parameter 1:  
Execute:  

```
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB* | grep -E "KERNELS|DRIVERS"
```

Configure the second KERNELS value from the output to Parameter 1:  
![image/image_3.png](image/image_3.png)

Modification method for Parameter 2:  
Execute:  
```
v4l2-ctl --list-devices
```
Output:    
![image/image_4.png](image/image_4.png)

Then, for the first camera of this USB, execute:  
```
udevadm info -a -n /dev/video* | grep -E "KERNELS|SUBSYSTEMS"
```
Configure the first KERNELS value from the output to Parameter 2:  
![image/image_5.png](image/image_5.png)  
Then copy the template file to the following location:  
```
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
```
Then load the configuration:  
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Dual Gripper USB Port Configuration
The final configuration is as shown in the figure:   
![image/image_6.png](image/image_6.png)

Parts to modify:    
![image/image_7.png](image/image_7.png)

First, plug in the left gripper and configure it according to the single gripper configuration method; then unplug the left gripper, plug in the right gripper, and configure it again according to the single gripper configuration method; finally, load the configuration.

### Multiple Gripper USB Port Configuration
Similarly, add configurations to 99-usb-serial.rules.

## SDK Installation
Execute the following command:
```
cd genrobot_controller_sdk
sudo install/install.sh
```

## Single Gripper Startup
```
cd genrobot_controller_sdk
source install/setup.bash
roslaunch robot_driver single_gripper_start.launch
```

After startup, three image windows will pop up, and the output topics are:
```
/camera/color/image_raw     # Center camera
/camera_1/color/image_raw   # Left-side camera
/camera_2/color/image_raw   # Right-side camera
/encoder                    # Actual gripper opening/closing distance feedback
/tactile/left               # Left-side tactile sensor of gripper
/tactile/right              # Right-side tactile sensor of gripper
/target_distance            # Gripper opening/closing distance command
```


### Start demo script to receive commands and control gripper opening/closing
```
# /target_distance : The input distance range is [0.0, 0.103], meaning it can open up to 10 cm.
python3 script/das_controller_infer.py
```

## Dual Gripper Startup
```
cd genrobot_controller_sdk
source install/setup.bash
roslaunch robot_driver dual_gripper_start.launch
```

After startup, six image windows will pop up, and the output topics are:
```
/left_gripper/camera/color/image_raw    # Left gripper center camera
/left_gripper/camera_1/color/image_raw  # Left gripper left-side camera
/left_gripper/camera_2/color/image_raw  # Left gripper right-side camera
/left_gripper/encoder                   # Left gripper actual opening/closing distance feedback
/left_gripper/tactile/left              # Left gripper left-side tactile sensor
/left_gripper/tactile/right             # Left gripper right-side tactile sensor
/left_gripper/target_distance           # Left gripper opening/closing distance command

/right_gripper/camera/color/image_raw   # Right gripper center camera
/right_gripper/camera_1/color/image_raw # Right gripper left-side camera
/right_gripper/camera_2/color/image_raw # Right gripper right-side camera
/right_gripper/encoder                  # Right gripper actual opening/closing distance feedback
/right_gripper/tactile/left             # Right gripper left-side tactile sensor
/right_gripper/tactile/right            # Right gripper right-side tactile sensor
/right_gripper/target_distance          # Right gripper opening/closing distance command
```

### Start demo script to receive commands and control gripper opening/closing. Left gripper and right gripper control commands:
```
# /left_gripper/target_distance and /right_gripper/target_distance : The input distance range is [0.0, 0.103], meaning it can open up to 10 cm.
python3 script/left_das_controller_infer.py
python3 script/right_das_controller_infer.py
```
