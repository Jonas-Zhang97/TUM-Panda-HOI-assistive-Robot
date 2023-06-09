cmake_minimum_required(VERSION 3.1.3)
project(gripper_motion)

find_package(catkin REQUIRED COMPONENTS
pluginlib
franka_gripper
std_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Franka REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gripper_motion
  CATKIN_DEPENDS
    roscpp 
    std_msgs
    franka_gripper
  DEPENDS
    system_lib
)

# those in the include in cpp will be found in the following directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIR}
  ${Franka_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME}_node src/application/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Franka_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})