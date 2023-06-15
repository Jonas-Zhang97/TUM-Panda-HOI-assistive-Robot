#include <arm_motion/grasp.h>

bool Grasp::openHand()
{
  hand_group.setStartStateToCurrentState();
  hand_group.setNamedTarget("open");

  bool hand_open_success = (hand_group.plan(hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!hand_open_success)
  {
      ROS_ERROR_STREAM("Unable to open the gripper");
      return false;
  }
  else
  {
      hand_group.move();
      ROS_INFO_STREAM("Hand opened successfully");
  }

  return true;
}

bool Grasp::closeHand()
{
  // This is implementation using moveit API
  hand_group.setJointValueTarget(hand_group.getNamedTargetValues("close"));
  
  bool hand_close_success = (hand_group.plan(hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!hand_close_success)
  {
      ROS_ERROR_STREAM("Unable to close the gripper");
      return false;
  }
  else
  {
    hand_group.move();
  }

  return true;
}

bool Grasp::preGraspApproach()
{
}

bool Grasp::toGraspPose()
{
}

bool Grasp::grasp()
{
  grasp_client.waitForServer();
    
  goal.width = 0.03;
  goal.speed = 0.01;
  goal.force = 20;
  goal.epsilon.inner = 0.05;
  goal.epsilon.outer = 0.05;
  
  grasp_client.sendGoal(goal);

  bool finished = grasp_client.waitForResult(ros::Duration(10.0));
  actionlib::SimpleClientGoalState state = grasp_client.getState();
  if (finished)
  {
    ROS_INFO("Grasp client finished with: %s", state.toString().c_str());
  }

  return true;
}

bool Grasp::homing()
{
  arm_group.setNamedTarget("ready");
  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

  bool arm_ready_success = (arm_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!arm_ready_success)
  {
    ROS_ERROR_STREAM("The panda arm is not able to move back to ready");
    return false;
  }
  else
  {
    arm_group.move();
    openHand();
  }

  return true;
}

void Grasp::init()
{
  target_object_pose_sub_ = nh_.subscribe("/target_object_pose", 1, &Grasp::GraspPoseCallback, this);

  grasp_done_pub_ = nh_.advertise<std_msgs::Bool>("/grasp_done", 1);

  arm_group.setPoseReferenceFrame("panda_link0");
}

/*
void Grasp::update()
{
  if (has_target_object_)
  {
    preGraspApproach();
    toGraspPose();

    grasp();

    grasp_done_.data = true;
    grasp_done_pub_.publish(grasp_done);
    has_target_object_ = false;
  }
}
*/

void Grasp::GraspPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  grasp_pose_ = *msg;
  has_target_object_ = true;
}