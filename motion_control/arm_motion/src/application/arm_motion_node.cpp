#include <arm_motion/arm_motion.h>

int main(int argc, char** argv)
{
  // Initialize the node
  ros::init(argc, argv, "motion_control_node");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create objects to get access to my libraries
  ArmMotion arm_motion;

  // Set parameters for Rviz
  namespace rvt = rviz_visual_tools;

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  arm_motion.visual_tools.loadRemoteControl();

  // Initialize the scene in Rviz
  arm_motion.visual_tools.deleteAllMarkers();
  arm_motion.PSI.removeCollisionObjects(arm_motion.PSI.getKnownObjectNames());
  arm_motion.visual_tools.publishText(text_pose, "Motion Visualization", rvt::WHITE, rvt::XLARGE);
  arm_motion.visual_tools.trigger();

  // Reset Karina position
  arm_motion.hand_open();
  arm_motion.visual_tools.prompt("Press 'next' to continue");
  
  arm_motion.hand_close();
  arm_motion.visual_tools.prompt("Press 'next' to continue");
  
  arm_motion.hand_open();
  arm_motion.visual_tools.prompt("Press 'next' to end the demo");
  
  arm_motion.PSI.removeCollisionObjects(arm_motion.PSI.getKnownObjectNames());

  // End the node
  ros::shutdown();
  return 0;
}
