# genrobot_controller_sdk

## 进入目录
cd genrobot_controller_sdk

## 配置USB接口
如图所示，修改config/99-usb-serial.rules文件![alt text](image.png)两个地方，然后拷贝至/etc/udev/rules.d/99-usb-serial.rules；
配置详细过程请参开说明书。

## 单爪启动
cd genrobot_controller_sdk
source install/setup.bash
roslaunch robot_driver single_gripper_start.launch

启动后会弹出三个图像窗口，输出topic有：
/camera/color/image_raw     #中间相机
/camera_1/color/image_raw   #左侧相机
/camera_2/color/image_raw   #右侧相机
/encoder                    #夹爪实际开合距离反馈
/tactile/left               #夹爪左侧触觉
/tactile/right              #夹爪右侧触觉
/target_distance            #夹爪开合距离指令


### 启动demo脚本接收指令控制夹爪开合
python3 script/das_controller_infer.py


## 双爪启动
cd genrobot_controller_sdk
source install/setup.bash
roslaunch robot_driver dual_gripper_start.launch

启动后会弹出六个图像窗口，输出topic有：
/left_gripper/camera/color/image_raw    #左夹爪中间相机
/left_gripper/camera_1/color/image_raw  #左夹爪左侧相机
/left_gripper/camera_2/color/image_raw  #左夹爪右侧相机
/left_gripper/encoder                   #左夹爪实际开合距离反馈
/left_gripper/tactile/left              #左夹爪左侧触觉
/left_gripper/tactile/right             #左夹爪右侧触觉
/left_gripper/target_distance           #左夹爪开合距离指令

/right_gripper/camera/color/image_raw   #右夹爪中间相机
/right_gripper/camera_1/color/image_raw #右夹爪左侧相机
/right_gripper/camera_2/color/image_raw #右夹爪右侧相机
/right_gripper/encoder                  #右夹爪实际开合距离反馈
/right_gripper/tactile/left             #右夹爪左侧触觉
/right_gripper/tactile/right            #右夹爪右侧触觉
/right_gripper/target_distance          #右夹爪开合距离指令

### 启动demo脚本接收指令控制夹爪开合，左夹爪和右夹爪控制指令：
python3 script/left_das_controller_infer.py
python3 script/right_das_controller_infer.py
