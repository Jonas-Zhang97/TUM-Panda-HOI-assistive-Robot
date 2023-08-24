#include <plane_segmentation/plane_segmentation.h>

void PlaneSegmentation::preProcessCloud()
{
  // Downsampling to make dense to sparse, save in ds_cloud
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(raw_cloud_);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);

  PointCloud::Ptr ds_cloud(new PointCloud);
  voxel_grid.filter(*ds_cloud);

  // Transform the point cloud to the ref_frame and store the result in transf_cloud
  CloudPtr transf_cloud(new PointCloud);
  pcl_ros::transformPointCloud(ref_frame_, *raw_cloud_, *transf_cloud, tfListener_);

  // Remove the point cloud above 0.4m to remove the robot gripper
  pcl::PassThrough<PointT> pass_through_z;
  pass_through_z.setInputCloud(transf_cloud);
  pass_through_z.setFilterFieldName("z");
  pass_through_z.setFilterLimits(0.0, 0.4);    // just for test
  pass_through_z.filter(*preprocessed_cloud_);
  
  // Filter the point cloud in y direction to remove the wall
  pcl::PassThrough<PointT> pass_through_x;
  pass_through_x.setInputCloud(preprocessed_cloud_);
  pass_through_x.setFilterFieldName("x");
  pass_through_x.setFilterLimits(-0.5, 0.8);
  pass_through_x.filter(*preprocessed_cloud_);
  
  // Filter the point cloud in x direction to remove the other table
  pcl::PassThrough<PointT> pass_through_y;
  pass_through_y.setInputCloud(preprocessed_cloud_);
  pass_through_y.setFilterFieldName("y");
  pass_through_y.setFilterLimits(-0.25, 0.3);
  pass_through_y.filter(*preprocessed_cloud_);
}

void PlaneSegmentation::segmentCloud()
{
  // Use RANSAC to segment the pointcloud
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01);
  seg.setProbability(0.7);
  seg.setInputCloud(preprocessed_cloud_);
  seg.segment(*inliers, *coefficients);

  // Inliers as plane cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(preprocessed_cloud_);
  extract.setIndices(inliers);
  extract.filter(*plane_cloud_);

  // Outliers as object cloud
  extract.setNegative(true);
  extract.filter(*objects_cloud_);

  // Check if the RANSAC is correct
  if(coefficients->values.empty())
  {
    ROS_ERROR_STREAM("RanSaC cannot segment the plane and object cloud");
  }

  // Extract the normal vector 'n' perpendicular to the plane and the scalar distance 'd'
  Eigen::Vector3f n(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  double d = coefficients->values[3];

  // Find transformation of table plane w.r.t. the world using n and d
  Eigen::Quaternionf Q_plane_base = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f(0, 0, 1)); // = ?
  Eigen::Vector3f t_plane_base = d*n;
  Eigen::Affine3f T_plane_base = Eigen::Affine3f::Identity();
  T_plane_base.rotate(Q_plane_base.toRotationMatrix());
  T_plane_base.translate(t_plane_base);

  // Transform the objects_cloud into the table frame
  CloudPtr transf_cloud(new PointCloud);
  pcl::transformPointCloud(*objects_cloud_, *transf_cloud, T_plane_base);

  // Filter everything directly below the table and above it (z > 0.02 && < 0.15)
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.00, 0.3);
  CloudPtr filterd_cloud(new PointCloud);
  pass.filter(*filterd_cloud);

  // Transform back to base_link frame using the inverse transformation
  pcl::transformPointCloud(*filterd_cloud, *objects_cloud_, T_plane_base.inverse());
}

void PlaneSegmentation::init()
{
  // Set frame and topic name
  ref_frame_ = "panda_link0";
  
  std::string point_cloud_topic;
  nh_.getParam("point_cloud_topic", point_cloud_topic);
  
  // Set subscriber and publishers
  point_cloud_sub_ = nh_.subscribe(point_cloud_topic, 1, &PlaneSegmentation::PointCloudCallback, this);

  // preprocessed_cloud_pub_ = nh_.advertise<PointCloud>("/preprocessed_cloud", 1);
  // plane_cloud_pub_ = nh_.advertise<PointCloud>("/table_cloud", 1);
  objects_cloud_pub_ = nh_.advertise<PointCloud>("/hoi/objects_cloud", 1);

  // Set pointers for pcl
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  plane_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  ROS_INFO_STREAM("Initialized: plane_segmentation_node");
}

void PlaneSegmentation::update()
{
  if(updated_)
  {
    preProcessCloud();
    segmentCloud();

    // preprocessed_cloud_pub_.publish(*preprocessed_cloud_);
    // plane_cloud_pub_.publish(*plane_cloud_);
    ROS_INFO_STREAM("Publishing object cloud");
    objects_cloud_pub_.publish(*objects_cloud_);

    updated_ = false;
  }
}

void PlaneSegmentation::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  updated_ = true;

  point_cloud_frame_ = msg->header.frame_id;

  pcl::fromROSMsg(*msg, *raw_cloud_);
}