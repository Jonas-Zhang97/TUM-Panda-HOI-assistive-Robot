import rospy

import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloudCallback(data):
  xyz = np.array([[0, 0, 0]])
  gen = pc2.read_points(data.data, skip_nans=True)
  int_data = list(gen)
  for x in int_data:
    xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)

def generator():
  rospy.init_node('npy_generator', anonymous=True)
  point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloudCallback)

  rospy.spin()

if __name__ == '__main__':
  try:
    generator()
  except rospy.ROSInterruptException:
    pass