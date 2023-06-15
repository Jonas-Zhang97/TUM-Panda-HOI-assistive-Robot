#include <object_labeling/object_labeling.h>

ObjectLabeling::ObjectLabeling(
    const std::string& objects_cloud_topic_, 
    const std::string& camera_info_topic,
    const std::string& camera_frame) :
  is_cloud_updated_(false),
  has_camera_info_(false),
  objects_cloud_topic_(objects_cloud_topic_),
  camera_info_topic_(camera_info_topic),
  camera_frame_(camera_frame),
  K_(Eigen::Matrix3d::Zero())
{
}

ObjectLabeling::~ObjectLabeling()
{
}

bool ObjectLabeling::initalize(ros::NodeHandle& nh)
{
  object_point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(objects_cloud_topic_, 100, &ObjectLabeling::cloudCallback, this);
  object_detections_sub_ = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 100, &ObjectLabeling::detectionCallback, this);
  camera_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_, 100, &ObjectLabeling::cameraInfoCallback, this);

  labeled_object_cloud_pub_ = nh.advertise<PointCloudl>("/labeled_objects", 10);
  text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",10);

  // init internal pointclouds for processing (again pcl uses pointers)
  object_point_cloud_.reset(new PointCloud);    // holds unlabled object point cloud
  labeled_point_cloud_.reset(new PointCloudl);  // holds labled object point cloud

  dict_["sports ball"] = 1;
  dict_["bottle"] = 2;
  dict_["cup"] = 3;
  dict_["banana"] = 4;
  dict_["apple"] = 5;
  dict_["bowl"] = 6;
  // ... bananna, cup, apple, ...

  return true;
}

void ObjectLabeling::update(const ros::Time& time)
{
  // camera info and point cloud available
  if(is_cloud_updated_ && has_camera_info_)
  {
    is_cloud_updated_ = false;

    // label the objects in pointcloud based on 2d bounding boxes 
    if(!labelObjects(object_point_cloud_, labeled_point_cloud_))
      return;

    labeled_object_cloud_pub_.publish(labeled_point_cloud_);

    text_marker_pub_.publish<visualization_msgs::MarkerArray>(text_markers_);

  }
}

bool ObjectLabeling::labelObjects(CloudPtr& input, CloudPtrl& output)
{
  pcl::EuclideanClusterExtraction<PointT> ec;
  
  // General settings
  ec.setClusterTolerance (0.1); // 10cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (500);

  pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree -> setInputCloud(input);
  ec.setSearchMethod (tree);

  ec.setInputCloud(input);

  // holds the extracted cluster indices (just a integer for identifiction)
  std::vector<pcl::PointIndices> cluster_indices;

  ec.extract(cluster_indices);
  // ROS_INFO_STREAM("Number of clusters = " << cluster_indices.size());

  std::vector<Eigen::Vector3d> centroids;

  std::vector<pcl::PointIndices>::const_iterator cit;
  for (cit = cluster_indices.begin(); cit != cluster_indices.end(); ++cit)
  {
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    std::vector<int>::const_iterator iterator;
    for (iterator = cit->indices.begin(); iterator != cit->indices.end(); ++iterator)
    {
      PointT point = input->points[*iterator];
      centroid[0] += point.x;
      centroid[1] += point.y;
      centroid[2] += point.z;
    }
    int num_points = cit->indices.size();
    centroid /= num_points;
    centroids.push_back(centroid);
  }

  tf::StampedTransform transform;
  try
  {
    tfListener_.lookupTransform(camera_frame_,input->header.frame_id , ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d T_base_camera; // = ?;
  tf::transformTFToEigen(transform, T_base_camera);

  std::vector<Eigen::Vector3d> centroids_camera;
  for (size_t i = 0; i < centroids.size(); ++i)
  {
    Eigen::Vector3d centroid_camera = T_base_camera * centroids[i];
    //ROS_INFO_STREAM("centroid cam pos: " << centroid_camera.transpose());

    centroids_camera.push_back(centroid_camera);
  }

  std::vector<Eigen::Vector2d> pixel_centroids; // = ?
  for (size_t i = 0; i < centroids_camera.size(); ++i)
  {
    Eigen::Vector3d centroid_3d = centroids_camera[i];
    Eigen::Vector3d pixel_coord = K_ * centroid_3d;
    Eigen::Vector2d pixel_coord_2d(pixel_coord[0] / pixel_coord[2], pixel_coord[1] / pixel_coord[2]);
    pixel_centroids.push_back(pixel_coord_2d);
    //ROS_INFO_STREAM("Pixel coord" << pixel_coord_2d.transpose());
  }

  std::vector<int> assigned_labels(cluster_indices.size(), 0);                  // lables of each centroid
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class names of each centroid

  // ROS_INFO_STREAM("Number of bounding boxes = " << detections_.size());
  for(size_t i = 0; i < detections_.size(); ++i)
  {
    // get the bounding box we want to find the closest cenroid 
    darknet_ros_msgs::BoundingBox& bounding_box = detections_[i];
    double bounding_box_center_x = (bounding_box.xmax + bounding_box.xmin) / 2;
    double bounding_box_center_y = (bounding_box.ymax + bounding_box.ymin) / 2;
    Eigen::Vector2d box_center(bounding_box_center_x, bounding_box_center_y);
    // ROS_INFO_STREAM("Box center: " << box_center.transpose());
    double minDistance = std::numeric_limits<double>::max(); 
    int match = -1; 

    for (size_t j = 0; j < pixel_centroids.size(); ++j) 
    { 
      //ROS_INFO_STREAM("pixel: " << pixel_centroids.size());
      
      double distance = std::sqrt((pixel_centroids[j][0]-bounding_box_center_x)*(pixel_centroids[j][0]-bounding_box_center_x) + (pixel_centroids[j][1]-bounding_box_center_y)*(pixel_centroids[j][1]-bounding_box_center_y));
     
      if (distance < minDistance) 
      {
        minDistance = distance;
        match = j;
      }
    }
    
    // remember the label of match
    if(dict_.find(bounding_box.Class) != dict_.end())
    {
      assigned_labels[match] = dict_[bounding_box.Class]; // set match to defined class index
      assigned_classes[match] = bounding_box.Class;       // set match to class name
      
    }
  }

  // ROS_INFO_STREAM("Assigned output: " << assigned_classes.size());

  // relabel the point cloud
  output->points.clear();
  output->header = input->header;

  PointTl pt;
  int i = 0;
  cit = cluster_indices.begin();
  for(; cit != cluster_indices.end(); ++cit, ++i ) 
  {
    // relabel all the points inside cluster
    std::vector<int>::const_iterator it = cit->indices.begin();
    for(; it != cit->indices.end(); ++it ) 
    {
      PointT& cpt = input->points[*it];
      pt.x = cpt.x;
      pt.y = cpt.y;
      pt.z = cpt.z;
      pt.label = assigned_labels[i];    // Note: To test clustering without the matching stuff just use pt.lable = i
      output->points.push_back( pt );
    }
  }

  // create a text marker that displays the assigned class name (assigned_classes) 
  // at the 3d position of the corresponding centroid
  text_markers_.markers.resize(assigned_classes.size());
  for(size_t i = 0; i < assigned_classes.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = assigned_classes[i];
    marker.pose.position.x = centroids[i][0];
    marker.pose.position.y = centroids[i][1];
    marker.pose.position.z = centroids[i][2] + 0.1;
    marker.color.a = 1.0;
    marker.scale.z = 0.1;
    marker.id = i;
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp = ros::Time::now();
    text_markers_.markers[i] = marker;
  }

  return true;
}


void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert to pcl
  is_cloud_updated_ = true;
  pcl::fromROSMsg(*msg, *object_point_cloud_);
}

void ObjectLabeling::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  detections_.clear();
  for (const darknet_ros_msgs::BoundingBox& b_box : msg->bounding_boxes)
  {
    detections_.push_back(b_box);
  }
}

void ObjectLabeling::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  // copy camera info
  has_camera_info_ = true;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  for(size_t i = 0; i < 9; ++i)
  {
    K(i) = msg->K[i];
  }
  K_ = K.transpose();
  //ROS_INFO_STREAM("K = " << K);
}