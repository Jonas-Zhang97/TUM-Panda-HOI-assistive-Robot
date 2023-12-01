#include <pick_place/place.h>

//TODO: padding the planning scene to reduce the posibility of collision

bool Place::init()
{
  command_sub_ = nh_.subscribe("/hoi/pick_done", 1, &Place::commandCallback, this);
  pose_sub_ = nh_.subscribe("/hoi/grasp_pose", 1, &Place::poseCallback, this);
  place_done_pub_ = nh_.advertise<std_msgs::Bool>("/hoi/place_done", 1);

  // Initialize the move_group
  arm_group.setPlannerId("RRTConnectkConfigDefault");
  arm_group.setPlanningTime(30.0);
  arm_group.setMaxAccelerationScalingFactor(0.1);
  arm_group.setMaxVelocityScalingFactor(0.1);
  arm_group.setEndEffectorLink("panda_hand");
  arm_group.setPoseReferenceFrame("panda_link0");

  const std::vector<double> open_value = {0.04, 0.04};
  gripper_group.rememberJointValues("open_complete", open_value);

  has_command_ = false;

  return true;
}

void Place::update()
{
  if (has_command_)
  {
    ros::Duration(1.0).sleep();
    
    // Add table collision model
    std::vector<double> table_pose = {0.50, 0.0, -0.01};
    std::vector<double> table_primitive = {1.0, 1.0, 0.02};
    std::string table_name = "table";

    std::vector<moveit_msgs::CollisionObject> table_model = generateCollisionModel(table_pose, table_primitive, table_name);
    PSI_.addCollisionObjects(table_model);

    // Perform place
    place();
    
    // Publish done flag
    std_msgs::Bool place_done;
    place_done.data = true;
    place_done_pub_.publish(place_done);

    // Clear all collision objects in the planning scene
    std::vector<std::string> object_names;
    object_names = PSI_.getKnownObjectNames();
    ROS_INFO_STREAM("Number of collision objects in the scene: " << object_names.size());
    PSI_.removeCollisionObjects(object_names);
    object_names.clear();

    // Reset local flag
    has_command_ = false;
  }
}

bool Place::place()
{
  // homing();
  if (prePlaceApproach() == -1)
  {
    return false;
  }

  visual_tools.prompt("press 'next' to continue.");
  
  if (toPlacePose() == -1)
  {
    return false;
  }

  visual_tools.prompt("press 'next' to continue.");

  if (openGripper() == -1)
  {
    return false;
  }

  visual_tools.prompt("press 'next' to continue.");

  if (postPlaceRetreat() == -1)
  {
    return false;
  }

  visual_tools.prompt("press 'next' to continue.");

  if (homing() == -1)
  {
    return false;
  }

  visual_tools.prompt("press 'next' to the next loop.");

  return true;
}

int Place::prePlaceApproach()
{
  // Add padding
  // double padding_center_height = 0.15;
  // std::vector<double> padding_pose = {0.4, 0.0, padding_center_height};
  // std::vector<double> padding_primitive = {0.1, 1.0, padding_center_height * 2};
  // std::string padding_name = "padding";

  // std::vector<moveit_msgs::CollisionObject> padding_model = generateCollisionModel(padding_pose, padding_primitive, padding_name);
  // PSI_.addCollisionObjects(padding_model);

  // Define the stage goal
  geometry_msgs::PoseStamped pre_approach_goal;
  pre_approach_goal.header.frame_id = "panda_link0";
  pre_approach_goal.pose.position.x = 0.6;
  pre_approach_goal.pose.position.y = 0;
  pre_approach_goal.pose.position.z = grasp_height_ + 0.1;
  pre_approach_goal.pose.orientation = grasp_orientation_;

  ROS_INFO_STREAM("Approaching pre-place pose: " << pre_approach_goal);

  // Plan
  arm_group.setStartStateToCurrentState();
  arm_group.setPoseTarget(pre_approach_goal);

  bool success = (arm_group.plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (!success)
  {
    ROS_ERROR_STREAM("Unable to plan: prePickApproach");
    return -1;
  }

  // Execute
  arm_group.move();

  // removeObject("padding");
  
  return 1;
}

int Place::toPlacePose()
{
  ROS_INFO_STREAM("Getting to the place pose");
  geometry_msgs::Pose place_pose;
  place_pose.position.x = 0.6;
  place_pose.position.y = 0;
  place_pose.position.z = grasp_height_+0.03;
  place_pose.orientation = grasp_orientation_;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(place_pose);

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

int Place::openGripper()
{
  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("open_complete");

  bool success = (gripper_group.plan(gripper_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (!success)
  {
    ROS_ERROR_STREAM("Unable to plan: prePickApproach");
    return -1;
  }

  // Execute
  gripper_group.move();
  
  return 1;
}

int Place::postPlaceRetreat()
{
  ROS_INFO_STREAM("Retreating to the post-pick pose");
  geometry_msgs::Pose post_retreat_goal;
  post_retreat_goal.position.x = 0.6;
  post_retreat_goal.position.y = 0;
  post_retreat_goal.position.z = grasp_height_ + 0.2;
  post_retreat_goal.orientation = grasp_orientation_;
  
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

int Place::homing()
{
  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("ready");

  bool success = (arm_group.plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (!success)
  {
    ROS_ERROR_STREAM("Unable to plan: homing");
    return -1;
  }

  // Execute
  arm_group.move();

  // removeObject("padding");
  
  return 1;
}

std::vector<moveit_msgs::CollisionObject> Place::generateCollisionModel(std::vector<double> &position_data, std::vector<double> &primitive_data, std::string &name)
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

void Place::removeObject(const std::string &name)
{
  std::vector<std::string> to_be_removed;
  to_be_removed.push_back(name);
  PSI_.removeCollisionObjects(to_be_removed);
}

void Place::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  grasp_orientation_ = msg->pose.orientation;
  grasp_height_ = msg->pose.position.z;
}

void Place::commandCallback(const std_msgs::BoolConstPtr &msg)
{
  has_command_ = msg->data;
}