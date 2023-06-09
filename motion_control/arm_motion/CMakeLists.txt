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
moveit_grasps
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

add_library(${PROJECT_NAME} src/arm_motion.cpp)

add_executable(${PROJECT_NAME}_node src/application/arm_motion_node.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Franka_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} arm_motion)