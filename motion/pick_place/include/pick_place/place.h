#ifndef PLACE_H
#define PLACE_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Bool.h>

// namespace rvt = rviz_visual_tools;

class Place
{
  private:
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;

    moveit_visual_tools::MoveItVisualTools visual_tools;

  public:
    Place(): arm_group("panda_arm"), gripper_group("panda_hand"), visual_tools("panda_link0") {};

  public:
    bool init();
    void update();

  /* Robot Motion */
  private:
    bool place();

  private:
    std::vector<moveit_msgs::CollisionObject> generateCollisionModel(std::vector<double> &position, std::vector<double> &primitive, std::string &name);
    void removeObject(const std::string &name);

  private:
    int prePlaceApproach();
    geometry_msgs::Quaternion curr_quaternion_;

    int toPlacePose();
    int openGripper();
    int postPlaceRetreat();
    int homing();

  /* ROS Communication */
  private:
    ros::NodeHandle nh_;

  private:
    ros::Subscriber pose_sub_;
    ros::Subscriber command_sub_;

    ros::Publisher place_done_pub_;

  private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    geometry_msgs::Quaternion grasp_orientation_;
    double grasp_height_;

    void commandCallback(const std_msgs::BoolConstPtr &msg);
    bool has_command_;

  /* MoveIt */
  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;
    moveit::planning_interface::PlanningSceneInterface PSI_;
};

#endif