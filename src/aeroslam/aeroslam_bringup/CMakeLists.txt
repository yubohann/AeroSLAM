cmake_minimum_required(VERSION 3.0.2)
project(aeroslam_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/position_command_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
