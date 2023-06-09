#ifndef GRIPPER_MOTION_H
#define GRIPPER_MOTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>

typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient;

class GripperMotion
{
  private:
    ros::NodeHandle nh_;

    // Action client
    GraspClient graspClient_;
    franka_gripper::GraspGoal goal_;
    
    // Get the infomation about the arm
    ros::Subscriber at_grasp_pose_sub_;
    ros::Subscriber obj_primitive_sub_;  // TBD

    // Publish the state of the Gripper
    ros::Publisher grasp_pub_;

  private:
    // Close the gripper
    void atGraspPoseCallback(const std_msgs::Bool::ConstPtr &msg);

  public:
    // public members
    GripperMotion(): graspClient_("/franka_gripper/grasp", true){};
    
    void init();
    void update();

  private:
    bool execute_ = false;
    void grasp();
    std_msgs::Bool success_;
};

#endif