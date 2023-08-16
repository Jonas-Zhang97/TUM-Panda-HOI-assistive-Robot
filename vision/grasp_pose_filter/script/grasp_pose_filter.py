import sys
import rospy

from geometry_msgs.msg import PoseStamped, Quaternion

from std_msgs.msg import Bool

import tf
from tf.transformations import quaternion_from_matrix as matrix2quaternion
from tf.transformations import quaternion_matrix as quaternion2matrix
from tf.transformations import euler_from_quaternion as quaternion2euler

import numpy as np

class pose_talker:
  def __init__(self):
    # Read the output of the contact graspnet
    self.pred_grasps_cam = None
    self.scores = None
    self.grasp_pose = None
    self.command = False

    # Initialize publisher
    self.command_sub = rospy.Subscriber("/hoi/pick_target_object", Bool, self.commandCallback)
    self.pose_pub = rospy.Publisher("/hoi/grasp_pose", PoseStamped, queue_size=1)

    rospy.loginfo("Ready to publish the grasp pose")

  ### Functionalities of the codebase ###
  def resultLoader(self):
    output_data = np.load("/home/franka/contact_graspnet/results/predictions_data.npz", allow_pickle=True)
    # Save the data as arrays
    self.pred_grasps_cam = output_data["pred_grasps_cam.npy"].item()[-1]
    self.scores = output_data["scores.npy"].item()[-1]

  def poseGenerator(self):     # Generate the grasp pose as a PoseStamped object
    # Find the transformation with highest score
    index_max_score = np.argmax(self.scores)
    T_grasp_cam = np.array(self.pred_grasps_cam[index_max_score])
    q_grasp_cam = matrix2quaternion(T_grasp_cam)

    # Listen to the trasnformation of the camera_color_optical_frame w.r.t panda_link0
    listener = tf.TransformListener()
    try:
      listener.waitForTransform("panda_link0", "camera_color_optical_frame", rospy.Time.now(), rospy.Duration.from_sec(60))
      (t_cam_base, q_cam_base) = listener.lookupTransform("panda_link0", "camera_color_optical_frame", rospy.Time.now())
      ang_cam_base = quaternion2euler(q_cam_base)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.logwarn("can't get tf information")
    
    r_cam_base = quaternion2matrix(q_cam_base)
    T_cam_base = np.array(r_cam_base).reshape(4, 4)
    T_cam_base[0, 3] = t_cam_base[0]
    T_cam_base[1, 3] = t_cam_base[1]
    T_cam_base[2, 3] = t_cam_base[2]
    
    T_grasp_base = np.matmul(T_cam_base, T_grasp_cam)

    q_grasp_base = np.array(matrix2quaternion(T_grasp_base))
    t_grasp_base = np.array([T_grasp_base[0, 3], T_grasp_base[1, 3], T_grasp_base[2, 3]])

    print(q_grasp_base)
    print(t_grasp_base)

    self.grasp_pose = PoseStamped()
    self.grasp_pose.pose.position.x = t_grasp_base[0]
    self.grasp_pose.pose.position.y = t_grasp_base[1]
    self.grasp_pose.pose.position.z = t_grasp_base[2]
    self.grasp_pose.pose.orientation.x = q_grasp_base[0]
    self.grasp_pose.pose.orientation.y = q_grasp_base[1]
    self.grasp_pose.pose.orientation.z = q_grasp_base[2]
    self.grasp_pose.pose.orientation.w = q_grasp_base[3]
  
  def commandCallback(self, data):
    self.command = data.data

def main(args):
  pt = pose_talker()

  rospy.init_node("grasp_pose_filter_node", anonymous = True)

  while not rospy.is_shutdown():
    if pt.command:
      # Perform calculations
      pt.resultLoader()
      pt.poseGenerator()

      # Publish pose
      rospy.loginfo("Publishing grasp pose")
      pt.pose_pub.publish(pt.grasp_pose)

      # Reset the flag
      pt.command = False

if __name__ == "__main__":
  main(sys.argv)