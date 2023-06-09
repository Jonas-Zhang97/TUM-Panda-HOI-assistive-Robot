#include <arm_motion/arm_motion.h>

bool ArmMotion::hand_open()
{
    hand_group.setStartStateToCurrentState();
    hand_group.setNamedTarget("open");

    moveit::planning_interface::MoveGroupInterface::Plan hand_open;

    bool hand_open_success = (hand_group.plan(hand_open) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!hand_open_success)
    {
        ROS_ERROR_STREAM("Unable to open the gripper");
        return false;
    }
    else
    {
        hand_group.asyncExecute(hand_open);
        ROS_INFO_STREAM("Hand opened successfully");
    }

    return true;
}

bool ArmMotion::hand_close_franka()
{
    const std::string Franka_IP = "10.162.83.121";
    franka::Gripper gripper(Franka_IP);
    
    double close_width = 0.01;
    double speed = 0.01;
    double max_close_force = 20.0; // force in Newton
    
    gripper.grasp(close_width, speed, max_close_force);

    return true;
}

bool ArmMotion::hand_close()
{
  // This is implementation using moveit classes
  hand_group.setNamedTarget("close");
  
  moveit::planning_interface::MoveGroupInterface::Plan hand_close;
  
  bool hand_close_success = (hand_group.plan(hand_close) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!hand_close_success)
  {
      ROS_ERROR_STREAM("Unable to close the gripper");
      return false;
  }
  else
  {    
        hand_group.asyncExecute();
        // ros::Duration(0.01).sleep();
        hand_group.stop();
        ROS_INFO_STREAM("Hand closed successfully");
    }

    return true;
}

bool ArmMotion::homing()
{
    arm_group.setNamedTarget("ready");
    moveit::planning_interface::MoveGroupInterface::Plan arm_ready;

    bool arm_ready_success = (arm_group.plan(arm_ready) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!arm_ready_success)
    {
        ROS_ERROR_STREAM("The panda arm is not able to move back to ready");
        return false;
    }
    else
    {
        arm_group.execute(arm_ready);
        
        if(!hand_close())
        {
            ROS_ERROR_STREAM("Arm homing successfully, but unable to close the gripper");
        }
        else
        {
            ROS_INFO_STREAM("Homing successfully");
        }
    }

    return true;
}

bool ArmMotion::arm_to_target_pose(geometry_msgs::PoseStamped& pose)
{
    arm_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan to_target_pose;

    bool success_to_target = (arm_group.plan(to_target_pose) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success_to_target)
    {
        ROS_INFO_STREAM("Unable to find a valid path");
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Path found, executing...");
        arm_group.execute(to_target_pose);
    }

    return true;
}
