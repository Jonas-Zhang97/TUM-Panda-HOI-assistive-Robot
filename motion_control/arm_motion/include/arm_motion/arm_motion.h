#ifndef ARM_MOTION_H
#define ARM_MOTION_H

#include <ros/ros.h>

#include <control_msgs/GripperCommand.h>

// Move group
#include <moveit/move_group_interface/move_group_interface.h>

// Planning scene
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Grasp
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>

// Visualize robot and trajectories
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// Object for Rviz
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Franka gripper
#include <franka_gripper/franka_gripper.h>

// Mathmatic
#include <std_msgs/Float64.h>
#include <cmath>

class ArmMotion
{
  private:
    /* 1. Variables for the planning groups */
    moveit::planning_interface::MoveGroupInterface hand_group;
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface manipulator_group;
  
  public:
    //
    moveit_visual_tools::MoveItVisualTools visual_tools;
    moveit::planning_interface::PlanningSceneInterface PSI;

    const double tau = 2*M_PI;
        
  public:
    /* Variables */
    /* 1. Initialize the planning groups and visual tools */
    ArmMotion():hand_group("panda_hand"), arm_group("panda_arm"), 
                manipulator_group("panda_manipulator"), visual_tools("panda_link0"){};
    
    /* Elemental Functions */
    /* 1. Hand motion */
    // Open the gripper with maximun width
    bool hand_open();
    // Close the gripper totally
    bool hand_close();
    /* 2 Arm motion */
    // Karina! Go home!
    bool homing();
    // Go! Karina! 
    bool arm_to_target_pose(geometry_msgs::PoseStamped& pose);

    /* Experimental features */

    // Using franka/Gripper class to close the gripper with certain force
    bool hand_close_franka();

    bool grasp();

    ros::NodeHandle nh;

  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
};

#endif