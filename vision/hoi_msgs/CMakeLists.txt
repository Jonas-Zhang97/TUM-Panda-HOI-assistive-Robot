cmake_minimum_required(VERSION 3.0.2)
project(hoi_msgs)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

add_message_files(
  FILES
  ImageBoxes.msg
)

generate_messages(
  DEPENDENCIES
    darknet_ros_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HOI_msgs
  CATKIN_DEPENDS 
    darknet_ros_msgs 
    geometry_msgs 
    message_generation 
    roscpp rospy 
    sensor_msgs 
    std_msgs 
    cv_bridge 
    message_runtime 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
