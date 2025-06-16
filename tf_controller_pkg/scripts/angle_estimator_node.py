#!/usr/bin/env python
import rospy
import math
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from tf_controller_pkg.msg import Float32Stamped

class AngleEstimatorNode:
    def __init__(self):
        rospy.init_node('angle_estimator_node')

        # Camera intrinsics (to be filled once)
        self.fx = None
        self.cx = None

        # Pixel smoothing
        self.theta_cam_filt = None
        self.alpha = 0.2

        # Subscribe to camera_info
        rospy.Subscriber(
            '/camera/color/camera_info',
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        # Subscribe to bounding_boxes
        rospy.Subscriber(
            '/darknet_ros/bounding_boxes',
            BoundingBoxes,
            self.boundingboxes_callback,
            queue_size=1
        )

        # Publisher for estimated angle
        self.angle_pub = rospy.Publisher(
            '/estimated_angle',
            Float32Stamped,
            queue_size=1
        )

        rospy.loginfo("AngleEstimatorNode initialized.")
        rospy.spin()

    def camera_info_callback(self, cam_info):
        # Only set intrinsics once
        if self.fx is None or self.cx is None:
            self.fx = cam_info.K[0]
            self.cx = cam_info.K[2]
            rospy.loginfo(
                "Camera intrinsics received: fx=%.2f, cx=%.2f",
                self.fx, self.cx
            )

    def boundingboxes_callback(self, boxes_msg):
        # Ensure we have intrinsics
        if self.fx is None or self.cx is None:
            rospy.logwarn_throttle(10.0, "Waiting for camera intrinsics.")
            return

        # No detections? log and return
        if not boxes_msg.bounding_boxes:
            rospy.loginfo_throttle(5.0, "No bounding boxes detected.")
            return

        # Choose the most confident detection
        best_box = max(
            boxes_msg.bounding_boxes,
            key=lambda b: b.probability
        )

        # Compute center pixel u-coordinate
        u_center = (best_box.xmin + best_box.xmax) / 2.0

        # Compute horizontal angle (radians)
        theta = math.atan((u_center - self.cx) / self.fx)

        # Smooth angle
        if self.theta_cam_filt is None:
            self.theta_cam_filt = theta

        else:
            self.theta_cam_filt = (
                self.alpha * theta
                + (1.0 - self.alpha) * self.theta_cam_filt
            )
 

        # Publish the angle in radians for downstream use
        angle_msg = Float32Stamped()
        angle_msg.header.stamp = boxes_msg.header.stamp
        angle_msg.data = self.theta_cam_filt
        self.angle_pub.publish(angle_msg)

        # Convert to degrees for logging without affecting theta
        angle_deg = math.degrees(self.theta_cam_filt)
#        rospy.loginfo_throttle(
#            0.5,
#           "%s detected (p=%.2f), angle=%.3f deg",
#            best_box.Class, best_box.probability, angle_deg
#        )

if __name__ == '__main__':
    try:
        AngleEstimatorNode()
    except rospy.ROSInterruptException:
        pass

