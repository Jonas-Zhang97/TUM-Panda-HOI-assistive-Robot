#include <gripper_motion/gripper_motion.h>

void GripperMotion::init()
{
  at_grasp_pose_sub_ = nh_.subscribe("/at_grasp_pose", 1, &GripperMotion::atGraspPoseCallback, this);
  grasp_pub_ = nh_.advertise<std_msgs::Bool>("/grasp_done", 1);

  success_.data = false;
}

void GripperMotion::update()
{
  if (execute_)
  {
    grasp();
    if (success_.data)
    {
      grasp_pub_.publish(success_);
      success_.data = false;
    }
  }
  execute_ = false;
}

void GripperMotion::grasp()
{
  graspClient_.waitForServer();
    
    goal_.width = 0.03;
    goal_.speed = 0.01;
    goal_.force = 20;
    goal_.epsilon.inner = 0.05;
    goal_.epsilon.outer = 0.05;
    
    graspClient_.sendGoal(goal_);

    bool finished = graspClient_.waitForResult(ros::Duration(10.0));
    actionlib::SimpleClientGoalState state = graspClient_.getState();
    if (finished)
    {
      ROS_INFO("Grasp client finished with: %s", state.toString().c_str());
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        success_.data = true;
      }
    }
}

void GripperMotion::atGraspPoseCallback(const std_msgs::Bool::ConstPtr &msg)
{
  execute_ = msg->data;
}