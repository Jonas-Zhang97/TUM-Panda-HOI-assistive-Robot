#ifndef GRASP_H
#define GRASP_H

#include <ros/ros.h>

#include <control_msgs/GripperCommand.h>

// Move group
#include <moveit/move_group_interface/move_group_interface.h>

// Planning scene
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Visualize robot and trajectories
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// Object for Rviz
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Grasp
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>

// Std_msgs for ROS communication
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// Mathmatic
#include <cmath>

typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient;

class Grasp
{
  private:
    GraspClient grasp_client;
    franka_gripper::GraspGoal goal;
  private:
    // Variables for the planning groups
    moveit::planning_interface::MoveGroupInterface hand_group;
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface manipulator_group;
  
  public:
    // For Planning scene
    moveit_visual_tools::MoveItVisualTools visual_tools;
    moveit::planning_interface::PlanningSceneInterface PSI;

    const double tau = 2*M_PI;
        
  public:
    // 1. Initialize the planning groups and visual tools
    Grasp():grasp_client("/franka_gripper/grasp", true), hand_group("panda_hand"), arm_group("panda_arm"), 
                manipulator_group("panda_manipulator"), visual_tools("panda_link0"){};
    
  public:
    bool openHand();
    bool closeHand();
    bool homing();

    bool preGraspApproach();
    bool toGraspPose();

    void grasp();

  private:
    ros::NodeHandle nh_;

  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan hand_plan;

  public:
    void init();
    void update();

  private:
    ros::Subscriber grasp_pose_sub_;
    void GraspPoseCallback(geometry_msgs::PoseStampedConstPtr& msg);

    ros::Publisher grasp_done_pub_;
};

#endif