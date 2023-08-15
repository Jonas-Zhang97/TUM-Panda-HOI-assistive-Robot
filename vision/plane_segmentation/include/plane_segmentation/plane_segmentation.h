#ifndef PLANE_SEGMENTATION_H
#define PLANE_SEGMENTATION_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

class PlaneSegmentation
{
  public:
    typedef pcl::PointXYZRGB PointT;                // The Point Type
    typedef pcl::PointCloud<PointT> PointCloud;     // The PointCloud Type
    typedef PointCloud::Ptr CloudPtr;               // The PointCloud Pointer Type

    typedef pcl::PointXYZ PointTG;                // The Point Type
    typedef pcl::PointCloud<PointTG> PointCloudG;     // The PointCloud Type
    typedef PointCloudG::Ptr CloudPtrG;               // The PointCloud Pointer Type

  private:
    void preProcessCloud();
    void segmentCloud();
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void generateDepthImage();
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void boxFilter();
    void boxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);

  public:
    void init();
    void update();

  private:
    std::string ref_frame_;
    std::string point_cloud_topic_;

    ros::NodeHandle nh_;

    ros::Subscriber point_cloud_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber bounding_box_sub_;

    ros::Publisher preprocessed_cloud_pub_;
    ros::Publisher plane_cloud_pub_;
    ros::Publisher objects_cloud_pub_;
    ros::Publisher objects_depth_image_pub_;

    CloudPtr raw_cloud_;                  //!< Inital raw point cloud
    CloudPtr preprocessed_cloud_;         //!< after preprocessing
    CloudPtr plane_cloud_;                //!< points of table surface
    CloudPtr objects_cloud_;              //!< points of objects
    
    tf::TransformListener tfListener_;    //!< access ros tf tree to get frame transformations

    bool updated_ = false;

  // The following part is added as version 0.1
  private:
    std::vector<int> image_size_;

    Eigen::Matrix3d K_;

    cv_bridge::CvImage depth_image_;

    std::vector<Eigen::Vector4d> boxes_;

  // The following part is added as version 0.2
  private:
    std::string point_cloud_frame_;
};

#endif