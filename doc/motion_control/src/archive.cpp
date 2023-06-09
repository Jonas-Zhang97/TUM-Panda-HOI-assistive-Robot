std::vector<moveit_msgs::CollisionObject> addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& PSI)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  /* The first object to avoid */
  collision_objects[0].id = "collision1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(2);
  collision_objects[0].primitives[0].dimensions[0] = 0.02;
  collision_objects[0].primitives[0].dimensions[1] = 0.01;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.01;
  collision_objects[0].primitive_poses[0].orientation.w = 1;

  collision_objects[0].operation = collision_objects[0].ADD;

  /* The second object to avoid */
  collision_objects[1].id = "collision2";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.01;
  collision_objects[1].primitives[0].dimensions[1] = 0.02;
  collision_objects[1].primitives[0].dimensions[2] = 0.03;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.3;
  collision_objects[1].primitive_poses[0].position.y = 0.4;
  collision_objects[1].primitive_poses[0].position.z = 0.015;
  collision_objects[1].primitive_poses[0].orientation.w = 1;

  collision_objects[1].operation = collision_objects[1].ADD;

  /* The third object to avoid */
  collision_objects[2].id = "collision3";
  collision_objects[2].header.frame_id = "panda_link0";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].SPHERE;
  collision_objects[2].primitives[0].dimensions.resize(1);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.3;
  collision_objects[2].primitive_poses[0].position.y = -0.4;
  collision_objects[2].primitive_poses[0].position.z = 0.01;
  collision_objects[2].primitive_poses[0].orientation.w = 1;

  collision_objects[2].operation = collision_objects[2].ADD;

  /* Target object*/
  collision_objects[3].id = "target";
  collision_objects[3].header.frame_id = "panda_link0";

  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(2);
  collision_objects[3].primitives[0].dimensions[0] = 0.25;
  collision_objects[3].primitives[0].dimensions[1] = 0.03;

  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.1;
  collision_objects[3].primitive_poses[0].position.y = sqrt(0.24);
  collision_objects[3].primitive_poses[0].position.z = 0.15;
  collision_objects[3].primitive_poses[0].orientation.w = 1;

  collision_objects[3].operation = collision_objects[3].ADD;

  // Apply the objects to the planning scene
  PSI.applyCollisionObjects(collision_objects);

  return collision_objects;
}

// Calculate the goal of grasp poses from target object
// This object contains 2 PoseStamped message, the first is pre-approach pose, the second is grasp pose
std::vector<geometry_msgs::PoseStamped> getPickPoses(std::vector<moveit_msgs::CollisionObject>& objects,
                                                           moveit::planning_interface::MoveGroupInterface& move_group_arm)
{
  // Find target object
  moveit_msgs::CollisionObject target_object;
  int n = objects.size();
  for (int i = 0; i<n; ++i)
  {
    // Find the target from the objects
    if (objects[i].id == "target")
    {
      target_object = objects[i];
    }
  }
  ROS_INFO_STREAM("Target object position: " << target_object.primitive_poses[0].position);

  // get the radius of object
  double r;
  if (target_object.primitives[0].dimensions.size() == 1)
  {
    r = target_object.primitives[0].dimensions[0];
    ROS_INFO_STREAM("Target object is sphere with radius: " << r);
  }
  else if (target_object.primitives[0].dimensions.size() == 2)
  {
    r = target_object.primitives[0].dimensions[1];
    ROS_INFO_STREAM("Target object is cylinder with radius: " << r);
  }
  else if (target_object.primitives[0].dimensions.size() == 3)
  {
    r = sqrt(pow(target_object.primitives[0].dimensions[0] / 2, 2)+pow(target_object.primitives[0].dimensions[1] / 2,2));
    ROS_INFO_STREAM("Target object is box with semi-radius:" << r);
  }

  // distance that should be maintained during the pre-approach procedure
  double dist_pre;
  dist_pre = r+0.17;

  // Get the current position of the panda_arm
  geometry_msgs::PoseStamped arm_curr_pose = move_group_arm.getCurrentPose();

  // Find the unit vector from target object to the start position of the end effector
  Eigen::Vector3d u_vec_t_e;
  Eigen::Vector3d target_position(target_object.primitive_poses[0].position.x,
                                  target_object.primitive_poses[0].position.y,
                                  target_object.primitive_poses[0].position.z);
  Eigen::Vector3d arm_curr_position(arm_curr_pose.pose.position.x,
                                    arm_curr_pose.pose.position.y,
                                    target_object.primitive_poses[0].position.z);
  u_vec_t_e = (arm_curr_position - target_position)/(arm_curr_position - target_position).norm();

  // Position
  Eigen::Vector3d pre_approach_position = target_position+dist_pre*u_vec_t_e;
  
  // Orientation
  tf2::Quaternion pre_approach_orientation;
  // Get theta
  Eigen::Vector3d e_y(0.0, 1.0, 0.0);
  Eigen::Vector3d u_vec = (target_position - pre_approach_position)/(target_position - pre_approach_position).norm();
  double theta = acos(u_vec.dot(e_y) / (u_vec.norm() * e_y.norm()));

  // Get the desired pre_approach pose
  geometry_msgs::PoseStamped pre_approach_pose;
  pre_approach_pose.header.frame_id = "panda_link0";
  pre_approach_pose.pose.position.x = pre_approach_position[0];
  pre_approach_pose.pose.position.y = pre_approach_position[1];
  pre_approach_pose.pose.position.z = pre_approach_position[2];
  pre_approach_orientation.setRPY(-tau/4, -tau/8, theta);
  pre_approach_pose.pose.orientation = tf2::toMsg(pre_approach_orientation);
  ROS_INFO_STREAM("pre approach pose = " << pre_approach_pose);

  // Distance that should be maintained during the grasp procedure
  Eigen::Vector3d dist_grasp = (0.075+r)*(-1*u_vec);

  // Get the grasp position
  Eigen::Vector3d grasp_position = target_position+dist_grasp;

  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header.frame_id = "panda_link0";
  grasp_pose.pose.position.x = grasp_position[0];
  grasp_pose.pose.position.y = grasp_position[1];
  grasp_pose.pose.position.z = grasp_position[2];
  grasp_pose.pose.orientation = pre_approach_pose.pose.orientation;
  ROS_INFO_STREAM("grasp pose = " << grasp_pose);

  std::vector<geometry_msgs::PoseStamped> pick_poses;
  pick_poses.resize(2);
  pick_poses[0] = pre_approach_pose;
  pick_poses[1] = grasp_pose;

  return pick_poses;
}

/* 4. Pick & Place */
void pick(moveit::planning_interface::MoveGroupInterface& move_group_arm,
          moveit::planning_interface::MoveGroupInterface& move_group_hand,
          std::vector<geometry_msgs::PoseStamped>& pick_poses)
{
  move_group_arm.setPoseTarget(pick_poses[0]);

  moveit::planning_interface::MoveGroupInterface::Plan pre_approach_plan;

  bool success_pap = (move_group_arm.plan(pre_approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success_pap)
  {
    ROS_INFO_STREAM("Pre-approach path found, executing...");
    // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
    // Note that this can lead to problems if the robot moved in the meanwhile.
    move_group_arm.execute(pre_approach_plan);
  }
  else
  {
    ROS_ERROR_STREAM("No valid path for pre-approach, abording...");
  }

  // Open the gripper
  openGripper(move_group_hand);

  move_group_arm.setPoseTarget(pick_poses[1]);

  moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;

  bool success_gp = (move_group_arm.plan(grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success_gp)
  {
    ROS_INFO_STREAM("Grasp path found, executing...");
    // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
    // Note that this can lead to problems if the robot moved in the meanwhile.
    move_group_arm.move();
  }
  else
  {
    ROS_ERROR_STREAM("No valid path for pre-approach, abording...");
  }
}

void place(moveit::planning_interface::MoveGroupInterface& move_group_arm,
           moveit::planning_interface::MoveGroupInterface& move_group_hand)
{
  /*TBD*/
}
