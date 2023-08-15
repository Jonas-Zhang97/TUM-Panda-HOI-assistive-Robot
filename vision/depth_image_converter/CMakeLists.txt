cmake_minimum_required(VERSION 3.0.2)
project(depth_image_converter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_geometry_msgs
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_image_converter
  CATKIN_DEPENDS 
    cv_bridge 
    roscpp 
    sensor_msgs 
    std_msgs 
    tf 
    tf2_geometry_msgs 
    tf_conversions
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME}_node src/application/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})