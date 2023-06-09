#include <plane_segmentation/plane_segmentation.h>

PlaneSegmentation::PlaneSegmentation(
    const std::string& pointcloud_topic, 
    const std::string& base_frame) :
  pointcloud_topic_(pointcloud_topic),
  base_frame_(base_frame),
  is_cloud_updated_(false)
{
}

PlaneSegmentation::~PlaneSegmentation()
{
}

bool PlaneSegmentation::initalize(ros::NodeHandle& nh)
{
  // Subscriber
  point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic_, 100, &PlaneSegmentation::cloudCallback, this);

  // Publisher
  plane_cloud_pub_ = nh.advertise<PointCloud>("/table_plane_pointcloud", 100);
  objects_cloud_pub_ = nh.advertise<PointCloud>("/object_pointcloud", 100);

  // Initalize pointers for pcl
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  plane_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  ROS_INFO_STREAM("Initialized");
  return true;
}

void PlaneSegmentation::update(const ros::Time& time)
{
  // update as soon as new pointcloud is available
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;
    // Apply pre-process cloud
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_))
    {
      return;
    }

    // Segment cloud into table and objects
    if(!segmentCloud(preprocessed_cloud_, plane_cloud_, objects_cloud_))
    {
      return;
    }

    // Publish both pointclouds obtained by segmentCloud()
    objects_cloud_pub_.publish(*objects_cloud_);
    plane_cloud_pub_.publish(*plane_cloud_);
  }
}

// Subsample and Filter the pointcloud
bool PlaneSegmentation::preProcessCloud(CloudPtr& input, CloudPtr& output)
{
  // Downsampling to make dense to sparse, save in ds_cloud
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(input);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);

  PointCloud::Ptr ds_cloud(new PointCloud);
  voxel_grid.filter(*ds_cloud);

  // Transform the point cloud to the base_frame and store the result in transf_cloud
  CloudPtr transf_cloud(new PointCloud);
  pcl_ros::transformPointCloud(base_frame_, *ds_cloud, *transf_cloud, tfListener_);

  // Trim points lower than z_min to remove the floor from the point cloud.
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud(transf_cloud);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(0.4, std::numeric_limits<float>::max());
  pass_through.filter(*output);

  return true;
}

// Remove every point that is not an object from the objects_cloud cloud
bool PlaneSegmentation::segmentCloud(CloudPtr& input, CloudPtr& plane_cloud, CloudPtr& objects_cloud)
{
  // Use RANSAC to segment the pointcloud
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // Set parameters of the SACS segmentation
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01);
  seg.setProbability(0.7);
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);


  // Inliers as plane cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.filter(*plane_cloud);

  // Outliers as object cloud
  extract.setNegative(true);
  extract.filter(*objects_cloud);

  // Check if the RANSAC is correct
  if(coefficients->values.empty())
  {
    return false;
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
  pcl::transformPointCloud(*objects_cloud, *transf_cloud, T_plane_base);

  // Filter everything directly below the table and above it (z > 0.01 && < 0.15)
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 0.15);
  CloudPtr filterd_cloud(new PointCloud);
  pass.filter(*filterd_cloud);

  // Transform back to base_link frame using the inverse transformation
  pcl::transformPointCloud(*filterd_cloud, *objects_cloud, T_plane_base.inverse());

  return true;
}

// Convert ros msg to pcl raw_cloud
void PlaneSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  is_cloud_updated_ = true;

  pcl::fromROSMsg(*msg, *raw_cloud_);
}
