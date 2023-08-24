import sys
import threading
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from std_msgs.msg import String

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
    self.image = None
    self.name = None

    self.command = False

    rgb_image_topic = rospy.get_param("rgb_image_topic")
    print(rgb_image_topic)
    camera_info_topic = rospy.get_param("camera_info_topic")
    print(camera_info_topic)

    self.depth_image_sub = rospy.Subscriber("/hoi/depth_image", Image, self.depthImageCallback)
    self.rgb_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgbImageCallback)
    self.K_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfoCallback)
    self.name_sub = rospy.Subscriber("/hoi/target_object_name", String, self.nameCallback)

  def depthImageCallback(self, data):
    # convert to npy
    # depth_image = data.depth_image
    try:
      rospy.loginfo("Data received")
      cv_depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
      self.depth = np.multiply(cv_depth_image, 10**(-3))     # size of the mat is: 720 x 1280
    except CvBridgeError as e:
      print("Recieved data type: ")
      print(type(data))
      rospy.logwarn("Unable to convert it to cv2")

    # save to .npy file
    # np.save("/home/franka/contact_graspnet/depth_image_data/depth.npy", cv_image)
  
  def rgbImageCallback(self, data):
    try:
      cv_rgb_image = self.bridge.imgmsg_to_cv2(data, "8UC3")
      self.image = cv_rgb_image
    except CvBridgeError as e:
      print("Recieved data type: ")
      print(type(data))
      rospy.logwarn("Unable to convert it to cv2")

  def cameraInfoCallback(self, data):
    self.cam_K = np.array(data.K).reshape([3, 3])

  def nameCallback(self, data):
    self.name = data.data

##### Main Function #####

def main(args):

  rospy.init_node("npy_generator_node", anonymous = True)
  rospy.loginfo("Initialized: npy_generator")

  dng = di_npy_generator()

  folder = rospy.get_param('npy_path')
  rospy.loginfo(".npy path: %s", folder)
  
  try:
    while not rospy.is_shutdown():
      if dng.depth is not None and dng.cam_K is not None:
        depth_data_dict = {"depth": dng.depth, "K": dng.cam_K, "rgb": dng.image}
        
        # Save the data dictionary to a .npy file
        file_name = dng.name
        extension = ".npy"
        path = folder+file_name+extension
        
        np.save(path, depth_data_dict)

        info_header = "saved to: "
        info_content = info_header + path
        rospy.loginfo(info_content)

        # Clear data, avoid unnecessary looping
        dng.depth = None
        dng.image = None
        dng.name = None

        rospy.sleep(1.0)
  except KeyboardInterrupt:
    rospy.loginfo("shut down")

if __name__ == "__main__":
  main(sys.argv)
