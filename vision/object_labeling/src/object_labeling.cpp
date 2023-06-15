#include <object_labeling/object_labeling.h>

void ObjectLabeling::init()
{
  cloud_updated_ = false;
  has_camera_info_ = false;

  camera_frame_ = "camera_depth_optical_frame";
  ref_frame_ = "panda_link0";
  objects_cloud_topic_ = "/objects_cloud";
  camera_info_topic_ = "/camera/depth/camera_info";
  bounding_boxes_topic_ = "/darknet_ros/bounding_boxes";

  object_detections_sub_ = nh_.subscribe(bounding_boxes_topic_, 100, 
                                         &ObjectLabeling::detectionCallback, this);
  object_point_cloud_sub_ = nh_.subscribe(objects_cloud_topic_, 100, 
                                          &ObjectLabeling::cloudCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 100,
                                   &ObjectLabeling::cameraInfoCallback, this);
  
  labeled_object_cloud_pub_ = nh_.advertise<PointCloudl>("/labeled_objects", 10);
  text_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/labels", 10);

  object_point_cloud_.reset(new PointCloud);    // holds unlabled object point cloud
  labeled_point_cloud_.reset(new PointCloudl);  // holds labled object point cloud

  // The following map is just for demo
  dict_["cup"] = 1;
  dict_["bowl"] = 2;
  dict_["bread"] = 3;
  dict_["apple"] = 4;
  dict_["bottle"] = 5;
  dict_["toilet"] = 6;
}

void ObjectLabeling::update()
{
  if (cloud_updated_ && has_camera_info_)
  {
    labelObjects();
    
    labeled_object_cloud_pub_.publish(*labeled_point_cloud_);
    text_marker_pub_.publish(text_markers_);
    
    cloud_updated_ = false;
  }
}

void ObjectLabeling::labelObjects()
{
  // Clustering
  pcl::EuclideanClusterExtraction<PointT> ec;

  ec.setClusterTolerance (0.1); // 10cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (500);

  pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree -> setInputCloud(object_point_cloud_);
  ec.setSearchMethod (tree);

  ec.setInputCloud(object_point_cloud_);

  std::vector<pcl::PointIndices> cluster_indices;

  ec.extract(cluster_indices);

  // Calculate the centroids of clustered clouds in ref_frame
  std::vector<Eigen::Vector3d> centroids;

  std::vector<pcl::PointIndices>::const_iterator cit;
  for (cit = cluster_indices.begin(); cit != cluster_indices.end(); ++cit)
  {
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    std::vector<int>::const_iterator iterator;
    for (iterator = cit->indices.begin(); iterator != cit->indices.end(); ++iterator)
    {
      PointT point = object_point_cloud_->points[*iterator];
      centroid[0] += point.x;
      centroid[1] += point.y;
      centroid[2] += point.z;
    }
    int num_points = cit->indices.size();
    centroid /= num_points;
    centroids.push_back(centroid);
  }

  // Get transformation of camera_frame _ w.r.t. ref_frame_
  tf::StampedTransform transform;
  try
  {
    tfListener_.lookupTransform(camera_frame_, ref_frame_, ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  Eigen::Affine3d T_base_camera;
  tf::transformTFToEigen(transform, T_base_camera);

  // Transform centroids into camera frame
  std::vector<Eigen::Vector3d> centroids_camera;
  for (size_t i = 0; i < centroids.size(); ++i)
  {
    Eigen::Vector3d centroid_camera = T_base_camera * centroids[i];
    centroids_camera.push_back(centroid_camera);
  }

  // Transform centroids into pixel frame
  std::vector<Eigen::Vector2d> pixel_centroids;
  for (size_t i = 0; i < centroids_camera.size(); ++i)
  {
    Eigen::Vector3d centroid_3d = centroids_camera[i];
    Eigen::Vector3d pixel_coord = K_ * centroid_3d;
    Eigen::Vector2d pixel_coord_2d(pixel_coord[0] / pixel_coord[2], pixel_coord[1] / pixel_coord[2]);
    pixel_centroids.push_back(pixel_coord_2d);
  }

  // Initialize the assignition
  std::vector<int> assigned_labels(cluster_indices.size(), 0);                  // lables of each centroid
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class names of each centroid

  // Assign labels
  for(size_t i = 0; i < detections_.size(); ++i)
  {
    // get the bounding box we want to find the closest cenroid 
    darknet_ros_msgs::BoundingBox& bounding_box = detections_[i];
    double bounding_box_center_x = (bounding_box.xmax + bounding_box.xmin) / 2;
    double bounding_box_center_y = (bounding_box.ymax + bounding_box.ymin) / 2;
    Eigen::Vector2d box_center(bounding_box_center_x, bounding_box_center_y);
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

  // relabel the point cloud
  labeled_point_cloud_->points.clear();
  labeled_point_cloud_->header = object_point_cloud_->header;

  PointTl pt;
  int i = 0;
  cit = cluster_indices.begin();
  for(; cit != cluster_indices.end(); ++cit, ++i ) 
  {
    // relabel all the points inside cluster
    std::vector<int>::const_iterator it = cit->indices.begin();
    for(; it != cit->indices.end(); ++it ) 
    {
      PointT& cpt = object_point_cloud_->points[*it];
      pt.x = cpt.x;
      pt.y = cpt.y;
      pt.z = cpt.z;
      pt.label = assigned_labels[i];    // Note: To test clustering without the matching stuff just use pt.lable = i
      labeled_point_cloud_->points.push_back( pt );
    }
  }

  // Create text markers
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
    marker.header.frame_id = object_point_cloud_->header.frame_id;
    marker.header.stamp = ros::Time::now();
    text_markers_.markers[i] = marker;
  }
}

void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  cloud_updated_ = true;
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
  has_camera_info_ = true;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  for(size_t i = 0; i < 9; ++i)
  {
    K(i) = msg->K[i];
  }
  K_ = K.transpose();
}