import sys
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class di_npy_generator:

  def __init__(self):
    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber("/object_depth_image", Image, self.imageCallback)
    self.K_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfoCallback)

  def imageCallback(self, data):
    # convert to npy
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      rospy.loginfo("Unable to convert sensor_msgs::Image to cv2")

    # save to .npy file
    np.save("/home/franka/contact_graspnet/depth_image_data/depth.npy", cv_image)

    rospy.sleep(1.0)

  def cameraInfoCallback(self, data):
    K = np.array(data.K).reshape([3, 3])
    # print("Camera Intrinsic Matrix K:")
    # print(K)

    np.save("/home/franka/contact_graspnet/depth_image_data/cam_K.npy", K)

class pc_npy_generator:

  def __init__(self):
    self.cloud_sub = rospy.Subscriber("/refined_objects_pointcloud", PointCloud2, self.cloudCallback)
  
  def cloudCallback(self, data):
    pc_data = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc_data))

    np.save("/home/franka/contact_graspnet/point_cloud_data/pointcloud.npy", points)

def main(args):
  from_point_cloud = rospy.get_param("~from_point_cloud")
  from_depth_image = rospy.get_param("~from_depth_image")
  
  if from_depth_image:
    dng = di_npy_generator()
  elif from_point_cloud:
    png = pc_npy_generator()
  else:
    rospy.logerr("please set the format of given input correctly")

  rospy.init_node("npy_generator_node", anonymous = True)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("shut down")

if __name__ == "__main__":
  main(sys.argv)