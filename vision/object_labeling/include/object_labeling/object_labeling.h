#ifndef OBJECT_LABELING_H
#define OBJECT_LABELING_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// darknet
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// std
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <map>

// pcl and opencv
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ObjectLabeling
{
  public:
    // pcl pointcloud types (only color RGB)
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef PointCloud::Ptr CloudPtr;

    // plc labled pointcloud types (color RGB + object labels L)
    typedef pcl::PointXYZL PointTl;
    typedef pcl::PointCloud<PointTl> PointCloudl;
    typedef PointCloudl::Ptr CloudPtrl;

    void init();
    void update();

  private:
    void labelObjects();
    void findMatch();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  private:
    bool cloud_updated_;                      //!< new pointcloud is recived
    bool has_camera_info_;                    //!< camera info recived
    std::string camera_frame_;                //!< camera frame name
    std::string ref_frame_;
    std::string objects_cloud_topic_;         //!< objects cloud topic name
    std::string camera_info_topic_;           //!< camera info topic name
    std::string bounding_boxes_topic_;

    Eigen::Matrix3d K_;                       //!< Camera matrix http://ksimek.github.io/2013/08/13/intrinsic/

    ros::NodeHandle nh_;
    ros::Subscriber object_detections_sub_;   //!< sub detections form detector
    ros::Subscriber object_point_cloud_sub_;  //!< sub point cloud from plane segmentation
    ros::Subscriber camera_info_sub_;         //!< sub camera info

    ros::Publisher labeled_object_cloud_pub_; //!< publisher for labeled pointcloud
    ros::Publisher text_marker_pub_;

    // outputs
    CloudPtrl labeled_point_cloud_;                 //!< labeled pointcloud (pointcloud that knows the object type)
    visualization_msgs::MarkerArray text_markers_;  //!< text markers for rviz

    // inputs 
    CloudPtr object_point_cloud_;                             //!< objects point cloud
    std::vector<darknet_ros_msgs::BoundingBox> detections_;   //!< vector of bounding boxes in 2d image

    tf::TransformListener tfListener_;        //!< access to tf tree for ros transformations

    std::map<std::string, int> dict_;         //!< mapping of object names to pointcloud label
};

#endif