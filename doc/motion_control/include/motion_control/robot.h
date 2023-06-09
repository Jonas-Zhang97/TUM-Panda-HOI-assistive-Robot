#ifndef robot_motion_H
#define robot_motion_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Grasp.h>
#include <franka_gripper/franka_gripper.h>

class RobotMotion
{
    private:
        /* 1. Variables for the planning groups */
        moveit::planning_interface::MoveGroupInterface hand_group;
        moveit::planning_interface::MoveGroupInterface arm_group;
        moveit::planning_interface::MoveGroupInterface manipulator_group;
        
    public:
        /* Variables */
        /* 1. Initialize the planning groups and visual tools */
        RobotMotion():hand_group("panda_hand"), arm_group("panda_arm"), 
                      manipulator_group("panda_manipulator"){};
        
        /* Functions */
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

        ros::NodeHandle nh;
        
        // Some public member value after the robot initialized, like move_group
        
};

#endif