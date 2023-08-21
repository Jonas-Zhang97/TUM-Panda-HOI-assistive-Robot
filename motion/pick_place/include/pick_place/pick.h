#ifndef PICK_H
#define PICK_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <actionlib/client/simple_action_client.h>

#include <franka_gripper/GraspAction.h>

class Pick
{
  public:
    typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient;
    const double tau = 2*M_PI;
  
  private:
    GraspClient grasp_client;
    franka_gripper::GraspGoal goal;

    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;

    moveit_visual_tools::MoveItVisualTools visual_tools;

  /* MoveIt */
  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;
    moveit::planning_interface::PlanningSceneInterface PSI_;

  public:
    Pick(): grasp_client("/franka_gripper/grasp", true), arm_group("panda_arm"), gripper_group("panda_hand"), visual_tools("panda_link0") {};

  public:
    bool init();
    void update();

  /* Robot Motion */
  private:
    bool pick();
  
  private:
    std::vector<moveit_msgs::CollisionObject> generateCollisionModel(std::vector<double> &position, std::vector<double> &primitive, std::string &name);
    void removeObject(const std::string &name);

  private:
    int homing();
    int openGripper();
    int prePickApproach();
    int toPickPose();
    int closeGripper();
    int postPickRetreat();

  /* ROS Communication */
  private:
    ros::NodeHandle nh_;

    ros::Subscriber pick_target_sub_;
    ros::Publisher pick_done_pub_;

  private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    geometry_msgs::PoseStamped target_pose_;
    bool has_target_;

};

#endif