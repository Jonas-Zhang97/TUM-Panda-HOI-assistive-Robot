cmake_minimum_required(VERSION 3.1.3)
project(motion_control)

find_package(catkin REQUIRED
  COMPONENTS
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
  LIBRARIES motion_control
  INCLUDE_DIRS include
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

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/robot.cpp src/scene.cpp)

add_executable(control src/application/motion_control.cpp)

target_link_libraries(control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(control motion_control)
target_link_libraries(control ${Franka_LIBRARIES})

install(TARGETS motion_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
