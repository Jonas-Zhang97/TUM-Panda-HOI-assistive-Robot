import rospy
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

if __name__ == '__main__':
    rospy.init_node('realsense_tf_broadcaster')

    tf_broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        transform = TransformStamped()
        transform.header.frame_id = "panda_link0"
        transform.child_frame_id = "camera_link"

        transform.transform.translation.x = 0.04
        transform.transform.translation.y = 0.7
        transform.transform.translation.z = 0.73

        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        quaternion = tft.quaternion_from_euler(roll, pitch, yaw)

        transform.transform.rotation = Quaternion(*quaternion)

        tf_broadcaster.sendTransformMessage(transform)

        rospy.sleep(0.1)