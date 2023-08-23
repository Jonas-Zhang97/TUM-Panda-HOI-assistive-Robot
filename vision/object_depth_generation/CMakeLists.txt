cmake_minimum_required(VERSION 3.0.2)
project(object_depth_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  darknet_ros_msgs
  image_geometry
  sensor_msgs
  tf
  tf_conversions
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_depth_generation
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    cv_bridge
    darknet_ros_msgs
    image_geometry
    sensor_msgs
    tf
    tf_conversions
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME}_node src/application/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})   

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
