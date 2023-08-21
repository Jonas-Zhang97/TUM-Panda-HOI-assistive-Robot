import sys
import rospy

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String

import tf
from tf.transformations import quaternion_from_matrix as matrix2quaternion
from tf.transformations import quaternion_matrix as quaternion2matrix
# from tf.transformations import euler_from_quaternion as quaternion2euler
from tf.transformations import quaternion_from_euler as euler2quaternion

import numpy as np

class pose_talker:
  def __init__(self):
    # Read the output of the contact graspnet
    self.pred_grasps_cam = None
    self.scores = None
    self.grasp_pose = None

    self.command = False
    self.name = None

    # Initialize publisher
    self.target_name_sub = rospy.Subscriber("/hoi/target_object", String, self.targetNameCallback)
    self.pose_pub = rospy.Publisher("/hoi/grasp_pose", PoseStamped, queue_size=1)

    rospy.loginfo("Ready to publish the grasp pose")

  ### Define the quaternion multiplication
  def quaternionMultiplication(self, q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    q_result = np.array([
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    
    return q_result

  ### Functionalities of the codebase ###
  def resultLoader(self):
    folder = "/home/franka/contact_graspnet/results/predictions_"
    name = self.name
    extension = ".npz"
    path = folder + name + extension
    output_data = np.load(path, allow_pickle=True)
    # Save the data as arrays
    self.pred_grasps_cam = output_data["pred_grasps_cam.npy"].item()[-1]
    self.scores = output_data["scores.npy"].item()[-1]

  def poseGenerator(self):     # Generate the grasp pose as a PoseStamped object
    # Find the transformation with highest score
    index_max_score = np.argmax(self.scores)
    T_grasp_cam = np.array(self.pred_grasps_cam[index_max_score])

    # Listen to the trasnformation of the camera_color_optical_frame w.r.t panda_link0
    listener = tf.TransformListener()
    try:
      listener.waitForTransform("panda_link0", "camera_color_optical_frame", rospy.Time.now(), rospy.Duration.from_sec(60))
      (t_cam_base, q_cam_base) = listener.lookupTransform("panda_link0", "camera_color_optical_frame", rospy.Time.now())
      # ang_cam_base = quaternion2euler(q_cam_base)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.logwarn("can't get tf information")
    
    r_cam_base = quaternion2matrix(q_cam_base)
    T_cam_base = np.array(r_cam_base).reshape(4, 4)
    T_cam_base[0, 3] = t_cam_base[0]
    T_cam_base[1, 3] = t_cam_base[1]
    T_cam_base[2, 3] = t_cam_base[2]
    
    T_grasp_base = np.matmul(T_cam_base, T_grasp_cam)

    # cgn means that the quaternion is the direct output of contact graspnet
    # to apply it on the robot, it should be rotated +90 degree around the z
    # axis of panda_link0
    q_grasp_base_cgn = np.array(matrix2quaternion(T_grasp_base))
    t_grasp_base = np.array([T_grasp_base[0, 3], T_grasp_base[1, 3], T_grasp_base[2, 3]])

    q_to_fit_panda = np.array(euler2quaternion(0, 0, 1.57))

    q_grasp_base = self.quaternionMultiplication(q_to_fit_panda, q_grasp_base_cgn)

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
  
  def targetNameCallback(self, data):
    self.name = data.data
    self.command = True

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
      pt.name = None

if __name__ == "__main__":
  main(sys.argv)