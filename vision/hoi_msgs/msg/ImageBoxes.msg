# This message encompasses the depth image along with its corresponding bounding boxes as extracted from darknet_ros.

# The rationale behind this message lies in the dynamic nature of the bounding boxes, which tend to fluctuate frequently. 
# Once the depth image undergoes filtration through these bounding boxes and subsequent processing to yield potential grasp 
# candidates, a consistent reference to the original bounding boxes becomes imperative. This continuity is essential for the 
# subsequent task of classifying the grasp candidates, attributing each to its respective object category.
sensor_msgs/Image depth_image

darknet_ros_msgs/BoundingBox[] bounding_boxes