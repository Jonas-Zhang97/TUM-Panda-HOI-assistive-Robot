cmake_minimum_required(VERSION 3.0.2)
project(npy_generator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES npy_generator
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS script/npy_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
