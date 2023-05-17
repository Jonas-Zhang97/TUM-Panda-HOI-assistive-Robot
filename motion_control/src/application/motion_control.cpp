#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/GripperCommand.h>

#include <cmath>

#include <motion_control/robot.h>
#include <motion_control/scene.h>

const double tau = 2*M_PI;

int main(int argc, char** argv)
{
  // Initialize the node
  ros::init(argc, argv, "motion_control_node");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create objects to get access to my libraries
  RobotMotion robot_motion;
  RvizScene rviz_scene;

  // Set parameters for Rviz
  namespace rvt = rviz_visual_tools;

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  rviz_scene.visual_tools.loadRemoteControl();

  // Initialize the scene in Rviz
  rviz_scene.visual_tools.deleteAllMarkers();
  
  rviz_scene.PSI.removeCollisionObjects(rviz_scene.PSI.getKnownObjectNames());
  
  rviz_scene.visual_tools.publishText(text_pose, "Motion Visualization", rvt::WHITE, rvt::XLARGE);

  rviz_scene.visual_tools.trigger();

  // Reset Karina position
  robot_motion.homing();

  rviz_scene.visual_tools.prompt("Press 'next' to continue");

  // Start motion
  robot_motion.hand_open();
  robot_motion.homing();

  rviz_scene.visual_tools.prompt("Press 'next' to end the demo");

  rviz_scene.PSI.removeCollisionObjects(rviz_scene.PSI.getKnownObjectNames());

  // End the node
  ros::shutdown();
  return 0;
}
