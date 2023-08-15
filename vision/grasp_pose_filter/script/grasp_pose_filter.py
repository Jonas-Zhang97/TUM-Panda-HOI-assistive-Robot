import sys
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool

from HOI_msgs.msg import ImageBoxes

import tf
import tf2_geometry_msgs as tf2_msgs

import numpy as np

class pose_talker:
  def __init__(self):
    # Read the output of the contact graspnet
    self.pred_grasps_cam = None
    self.scores = None
    self.contact_pts = None
  
  ### Functionalities of the codebase ###
  def resultLoader(self):
    output_data = np.load("/home/franka/contact_graspnet/results/predictions_data.npz", allow_pickle=True)
    # Save the data as arrays
    self.pred_grasps_cam = output_data["pred_grasps_cam.npy"].item()[-1]
    self.scores = output_data["scores.py"].item()[-1]
    self.contact_pts = output_data["contact_pts"].item[-1]