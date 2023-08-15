cmake_minimum_required(VERSION 3.0.2)
project(plane_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  roscpp
  std_msgs
  tf
  tf_conversions
  darknet_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plane_segmentation
  CATKIN_DEPENDS cv_bridge image_geometry image_transport roscpp std_msgs tf
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/plane_segmentation.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/applications/plane_segmentation_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  plane_segmentation
)
