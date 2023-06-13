cmake_minimum_required(VERSION 3.1.3)
project(arm_motion)

find_package(catkin REQUIRED COMPONENTS
interactive_markers
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
moveit_ros_perception
rviz_visual_tools
moveit_visual_tools
pluginlib
geometric_shapes
pcl_ros
pcl_conversions
rosbag
tf2_ros
tf2_eigen
tf2_geometry_msgs
franka_gripper
std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Franka REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_motion
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    franka_gripper
    std_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

add_library(grasp src/grasp.cpp)
add_library(place src/place.cpp)
add_library(common src/common.cpp)

add_executable(grasp_node src/applications/grasp_node.cpp)
add_executable(place_node src/applications/place_node.cpp)

target_link_libraries(grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Franka_LIBRARIES})
target_link_libraries(place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(common ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(grasp_node ${catkin_LIBRARIES} grasp common)
target_link_libraries(place_node ${catkin_LIBRARIES} place common)