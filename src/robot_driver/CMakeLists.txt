cmake_minimum_required(VERSION 3.0.2)
project(robot_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge
)

# Install Python executables
install(PROGRAMS
  scripts/camera_view_single.py
  scripts/databus_single.py
  scripts/tactile_dual_print.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/pack.py
  scripts/das_protocol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)