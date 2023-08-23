cmake_minimum_required(VERSION 3.0.2)
project(grasp_pose_filter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  tf
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grasp_pose_filter
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS script/grasp_pose_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
