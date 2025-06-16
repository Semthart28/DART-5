#!/usr/bin/env python
import rospy
import math
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from tf_controller_pkg.msg import Float32Stamped

class DistanceEstimatorNode:
    def __init__(self):
        rospy.init_node('distance_estimator_node')

        # Camera resolution (to be filled once)
        self.width = None
        self.height = None
        self.area_cam = None

        # Subscribe to camera_info
        rospy.Subscriber(
            '/camera/color/camera_info',
            CameraInfo,
            self.camera_resolution_callback,
            queue_size=1
        )

        # Subscribe to bounding_boxes
        rospy.Subscriber(
            '/darknet_ros/bounding_boxes',
            BoundingBoxes,
            self.boxarea_callback,
            queue_size=1
        )


        # Publisher for estimated distance
        self.edistance_pub = rospy.Publisher(
            '/estimated_distance',
            Float32Stamped,
            queue_size=1
        )

        rospy.loginfo("DistanceEstimatorNode initialized.")
        rospy.spin()

    def camera_resolution_callback(self, cam_info):
        # Only set intrinsics once
        if self.width is None or self.height is None:
            self.width = cam_info.width
            self.height = cam_info.height
            rospy.loginfo(
                "Camera resolution received: width=%.2f x height=%.2f pixels",
                self.width, self.height
            )

        if self.area_cam is None and self.width is not None and self.height is not None:
            self.area_cam = (self.width)*(self.height)
            rospy.loginfo(
                "Camera area calculated: area=%.2f pixels^2",
                self.area_cam
            )

    def boxarea_callback(self, boxes_msg):
        # Ensure we have intrinsics
        if self.area_cam is None:
            rospy.logwarn_throttle(10.0, "Waiting for area of camera")
            return

        # No detections? log and return
        if not boxes_msg.bounding_boxes:
            rospy.loginfo_throttle(5.0, "No bounding boxes detected")

        else: 
            # Choose the most confident detection
            best_box = max(
                boxes_msg.bounding_boxes,
                key=lambda b: b.probability
            )

            # Compute normalized box area (multiplied by 1000)
            box_area_norm = (float((best_box.xmax - best_box.xmin)*(best_box.ymax - best_box.ymin))) / (self.area_cam)
            box_area_norm_1000 = 1000*box_area_norm

            # Estimate distance using formula 
            estimated_distance = (0.8967)*(box_area_norm_1000**-0.468)

            # Publish the estimated distance
            edistance_msg = Float32Stamped()
            edistance_msg.header.stamp = boxes_msg.header.stamp
            edistance_msg.data = estimated_distance
            self.edistance_pub.publish(edistance_msg)

#            # Logging 
#            rospy.loginfo_throttle(
#                1,
#                "estimated_distance=%.2f m",
#                estimated_distance
#            )


if __name__ == '__main__':
    try:
        DistanceEstimatorNode()
    except rospy.ROSInterruptException:
        pass

















