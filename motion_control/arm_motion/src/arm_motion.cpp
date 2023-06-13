#include <arm_motion/arm_motion.h>

void ArmMotion::init()
{

}

bool ArmMotion::openHand()
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

bool ArmMotion::closeHand()
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

bool ArmMotion::homing()
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

