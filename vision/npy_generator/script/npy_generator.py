import sys
import threading
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# from hoi_msgs.msg import ImageBoxes

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

mutex = threading.Lock()

##### For Depth Image #####

class di_npy_generator:

  def __init__(self):
    self.bridge = CvBridge()

    self.cam_K = None
    self.depth = None

    self.command = False

    self.image_sub = rospy.Subscriber("/hoi/depth_image", Image, self.imageCallback)
    self.K_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfoCallback)

  def imageCallback(self, data):
    # convert to npy
    # depth_image = data.depth_image
    try:
      rospy.loginfo("Data received")
      cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
      self.depth = np.multiply(cv_image, 10**(-3))     # size of the mat is: 720 x 1280
    except CvBridgeError as e:
      print("Recieved data type: ")
      print(type(data))
      rospy.logwarn("Unable to convert it to cv2")

    # save to .npy file
    # np.save("/home/franka/contact_graspnet/depth_image_data/depth.npy", cv_image)

  def cameraInfoCallback(self, data):
    self.cam_K = np.array(data.K).reshape([3, 3])

##### For Point Cloud #####

class pc_npy_generator:

  def __init__(self):
    self.point = None
    # self.K = None

    self.cloud_sub = rospy.Subscriber("/objects_cloud", PointCloud2, self.cloudCallback)
    self.K_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cameraInfoCallback)

  def cloudCallback(self, data):
    pc_data = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc_data))

    self.point = points

  def cameraInfoCallback(self, data):
    self.cam_K = np.array(data.K).reshape([3, 3])

##### Main Function #####

def main(args):
  dng = di_npy_generator()
  png = pc_npy_generator()

  rospy.init_node("npy_generator_node", anonymous = True)
  
  try:
    while not rospy.is_shutdown():
      if dng.depth is not None and dng.cam_K is not None:
        depth_data_dict = {"depth": dng.depth, "K": dng.cam_K}
        # Save the data dictionary to a .npy file
        np.save("/home/franka/contact_graspnet/depth_image_data/data.npy", depth_data_dict)

        rospy.loginfo("Depth image data saved to depth_image_data/data.npy")

        # Clear data, avoid unnecessary looping
        dng.depth = None

        rospy.sleep(1.0)
  except KeyboardInterrupt:
    rospy.loginfo("shut down")

#  try:
#    while not rospy.is_shutdown() and png.command:
#      if png.point is not None and png.cam_K is not None:
#        point_data_dict = {"xzy": png.point, "K": png.cam_K}
#        # Save the data dictionary to a .npy file
#        np.save("/home/franka/contact_graspnet/point_cloud_data/data.npy", point_data_dict)
#
#        rospy.loginfo("Point cloud data saved to point_cloud_data/data.npy")
#
#        rospy.sleep(1.0)
#  except KeyboardInterrupt:
#    rospy.loginfo("shut down")

if __name__ == "__main__":
  main(sys.argv)