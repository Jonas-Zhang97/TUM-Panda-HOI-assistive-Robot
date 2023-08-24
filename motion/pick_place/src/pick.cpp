#include <pick_place/pick.h>

bool Pick::init()
{
  // Initialize flag
  has_target_ = false;

  // Subscribers and publishers
  pick_target_sub_ = nh_.subscribe("/hoi/grasp_pose", 1, &Pick::poseCallback, this);
  pick_done_pub_ = nh_.advertise<std_msgs::Bool>("/hoi/pick_done", 1);

  // Initialize the move_group
  arm_group.setPlannerId("RRTConnectkConfigDefault");
  arm_group.setPlanningTime(30.0);
  arm_group.setMaxAccelerationScalingFactor(0.3);
  arm_group.setMaxVelocityScalingFactor(0.3);
  arm_group.setEndEffectorLink("panda_hand");
  arm_group.setPoseReferenceFrame("panda_link0");

  const std::vector<double> open_value = {0.04, 0.04};
  gripper_group.rememberJointValues("open_complete", open_value);

  return true;
}

void Pick::update()
{
  if (has_target_)
  {
    // Add table to planning scene
    std::vector<double> table_pose = {0.50, 0.0, -0.01};
    std::vector<double> table_primitive = {1.0, 1.0, 0.02};
    std::string table_name = "table";

    std::vector<moveit_msgs::CollisionObject> table_model = generateCollisionModel(table_pose, table_primitive, table_name);
    PSI_.addCollisionObjects(table_model);

    pick();

    // Publish done flag
    std_msgs::Bool pick_done;
    pick_done.data = true;
    pick_done_pub_.publish(pick_done);

    // Reset local flag
    has_target_ = false;

    // Clear all collision objects in the planning scene
    std::vector<std::string> object_names;
    object_names = PSI_.getKnownObjectNames();
    ROS_INFO_STREAM("Number of collision objects in the scene: " << object_names.size());
    PSI_.removeCollisionObjects(object_names);
    object_names.clear();
  }
}

bool Pick::pick()
{
  // homing();
  if (openGripper() == -1)
  {
    return false;
  }
  
  if (prePickApproach() == -1)
  {
    return false;
  }

  if (toPickPose() == -1)
  {
    return false;
  }

  if (closeGripper() == -1)
  {
    return false;
  }

  if (postPickRetreat() == -1)
  {
    return false;
  }

  return true;
}

int Pick::openGripper()
{
  ROS_INFO_STREAM("Opening the gripper");
  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("open_complete");
  
  bool success = (gripper_group.plan(gripper_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success)
  {
    ROS_ERROR_STREAM("Unable to plan: openGripper");
    return -1;
  }
  gripper_group.move();
  return 1;
}

int Pick::prePickApproach()
{
  double padding_center_height = (target_pose_.pose.position.z - 0.12) / 2;
  std::vector<double> padding_pose = {0.4, 0.0, padding_center_height};
  std::vector<double> padding_primitive = {0.4, 1.0, padding_center_height * 2};
  std::string padding_name = "padding";

  std::vector<moveit_msgs::CollisionObject> padding_model = generateCollisionModel(padding_pose, padding_primitive, padding_name);
  PSI_.addCollisionObjects(padding_model);

  ROS_INFO_STREAM("Approaching pre-pick pose");
  // Get the pre-pick goal
  geometry_msgs::PoseStamped pre_approach_goal = target_pose_;
  pre_approach_goal.header.frame_id = "panda_link0";
  pre_approach_goal.pose.position.z += 0.1;

  arm_group.setStartStateToCurrentState();
  arm_group.setPoseTarget(pre_approach_goal);

  bool success = (arm_group.plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (!success)
  {
    ROS_ERROR_STREAM("Unable to plan: prePickApproach");
    return -1;
  }
  arm_group.move();

  removeObject("padding");
  
  return 1;
}

int Pick::toPickPose()
{
  ROS_INFO_STREAM("Getting to the pick pose");
  geometry_msgs::Pose pick_pose = target_pose_.pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pick_pose);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction >= 0.8)
  {
    arm_group.execute(trajectory);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to compute more than 0.8 of the whole trajectory, done: " << fraction);
    return -1;
  }
  return 1;
}

int Pick::closeGripper()
{
  ROS_INFO_STREAM("Closing the gripper");
  grasp_client.waitForServer();
    
  goal.width = 0.005;
  goal.speed = 0.1;
  goal.force = 20;
  goal.epsilon.inner = 0.5;
  goal.epsilon.outer = 0.5;
  
  grasp_client.sendGoal(goal);

  bool finished = grasp_client.waitForResult(ros::Duration(10.0));
  actionlib::SimpleClientGoalState state = grasp_client.getState();
  if (finished)
  {
    ROS_INFO("Grasp client finished with: %s", state.toString().c_str());
  }
  else
  {
    ROS_ERROR_STREAM("Failed to grasp the object");
    return -1;
  }

  return 1;
}

int Pick::postPickRetreat()
{
  ROS_INFO_STREAM("Retreating to the post-pick pose");
  geometry_msgs::Pose post_retreat_goal = target_pose_.pose;
  post_retreat_goal.position.z += 0.1;
  
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(post_retreat_goal);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction >= 0.8)
  {
    arm_group.execute(trajectory);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to compute more than 0.8 of the whole trajectory, done: " << fraction);
    return -1;
  }

  return 1;
}

std::vector<moveit_msgs::CollisionObject> Pick::generateCollisionModel(std::vector<double> &position_data, std::vector<double> &primitive_data, std::string &name)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";

  collision_object.id = name;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = primitive_data[0];
  primitive.dimensions[primitive.BOX_Y] = primitive_data[1];
  primitive.dimensions[primitive.BOX_Z] = primitive_data[2];

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = position_data[0];
  box_pose.position.y = position_data[1];
  box_pose.position.z = position_data[2];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  return collision_objects;
}

void Pick::removeObject(const std::string &name)
{
  std::vector<std::string> to_be_removed;
  to_be_removed.push_back(name);
  PSI_.removeCollisionObjects(to_be_removed);
}

void Pick::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // Save the received message
  target_pose_ = *msg;
  
  ROS_INFO_STREAM("Pose received");
  // Set the flag
  has_target_ = true;
}