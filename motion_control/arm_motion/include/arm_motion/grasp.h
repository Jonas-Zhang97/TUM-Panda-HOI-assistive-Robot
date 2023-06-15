#ifndef GRASP_H
#define GRASP_H

#include <ros/ros.h>

// Move group
#include <moveit/move_group_interface/move_group_interface.h>

// Planning scene
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/*
// Visualize robot and trajectories
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
*/

// Object for Rviz
// #include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Grasp
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>

// Std_msgs for ROS communication
#include <std_msgs/Bool.h>
// #include <std_msgs/Float64.h>

// Mathmatic
#include <cmath>

typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient;
const double tau = 2*M_PI;

class Grasp
{
  private:
    GraspClient grasp_client;
    franka_gripper::GraspGoal goal;
    
    // Variables for the planning groups
    moveit::planning_interface::MoveGroupInterface hand_group;
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface manipulator_group;
  
  public:
    // For Planning scene
    moveit_visual_tools::MoveItVisualTools visual_tools;
    moveit::planning_interface::PlanningSceneInterface PSI;
        
  public:
    // 1. Initialize the planning groups and visual tools
    Grasp():grasp_client("/franka_gripper/grasp", true), hand_group("panda_hand"), arm_group("panda_arm"), 
                manipulator_group("panda_manipulator"), visual_tools("panda_link0"){};
    
  private:
    bool openHand();
    bool closeHand();
    bool homing();

  public:
    bool preGraspApproach();
    bool toGraspPose();
    bool grasp();

  private:
    ros::NodeHandle nh_;

    ros::Subscriber target_object_pose_sub_;
    void GraspPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::Publisher grasp_done_pub_;
  
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan hand_plan;

  public:
    void init();
    void update();

  private:
    geometry_msgs::PoseStamped grasp_pose_;
    bool has_target_object_;

    std_msgs::Bool grasp_done_;

    geometry_msgs::PoseStamped pre_grasp_approach_;
};

#endif