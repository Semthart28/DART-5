#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from tf_controller_pkg.msg import Float32Stamped, Float32MultiArrayStamped
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

class TrafPerceptionNode:
    def __init__(self):  
        rospy.init_node('traf_perception_node')

        # Distance and angle margins
        self.distance_margin = 0.5           # meter
        self.angle_margin = math.radians(10)     # rad

        # Timeout for locking onto LiDAR match before reverting to estimated angle
        self.match_timeout = rospy.get_param('~match_timeout', 1.0)  # seconds
        
        # Horizontal and vertical distances between camera and LiDAR
        self.vx = -0.012                      # meter
        self.vy = 0.009                       # meter 

        # State for locking onto LiDAR measurements
        self.locked = False
        self.locked_theta = None
        self.locked_r = None
        self.last_match_time = rospy.Time.now()
 
        # Subscribe to estimated distance, estimated angle and laserscan using message_filters 
        self.sub_dist  = Subscriber('/estimated_distance', Float32Stamped)
        self.sub_angle = Subscriber('/estimated_angle', Float32Stamped)
        self.sub_scan  = Subscriber('/scan', LaserScan)
      
        # Time-sync the incoming topics using ApproximateTimeSynchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_dist, self.sub_angle, self.sub_scan],
            queue_size=10,
            slop=0.1  # seconds
        )
        self.ts.registerCallback(self.callback)

        # Publish [horizontal distance, vertical distance]
        self.pub = rospy.Publisher(
            '/traf_distances',
            Float32MultiArrayStamped,
            queue_size=5
        )
       
        rospy.loginfo("TrafPerceptionNode initialized with margins: dist=%.2f m, angle=%.2f deg", 
                      self.distance_margin, math.degrees(self.angle_margin))
        rospy.loginfo("Using sensor offset vx=%.2f m, vy=%.2f m", self.vx, self.vy)
        rospy.spin()

    def callback(self, dist_msg, angle_msg, scan_msg):
        # Current time
        now = rospy.Time.now()

        # Determine target angle & range: either locked LiDAR values or fall back to camera estimate
        if self.locked and (now - self.last_match_time).to_sec() <= self.match_timeout:
            target_theta = self.locked_theta
            target_r = self.locked_r

            theta_lidar = target_theta
            d_lidar = target_r
        else:
            d_est = dist_msg.data                                     # meter
            theta_cam = angle_msg.data                                # rad

            # Convert theta_cam and d_est to theta_lidar and d_lidar
            x_rel = d_est * math.sin(theta_cam) - self.vx             # meter
            y_rel = d_est * math.cos(theta_cam) - self.vy             # meter
            theta_lidar = math.atan2(x_rel, y_rel)                    # rad 
            d_lidar = math.hypot(x_rel, y_rel)                        # meter 

        # Search the LiDAR scan around d and theta
        matches = []
        for i, r in enumerate(scan_msg.ranges):
            if math.isinf(r) or math.isnan(r) or r == 0.0:
                continue
            beam_angle = scan_msg.angle_max - i * scan_msg.angle_increment
            if abs(beam_angle - theta_lidar) <= self.angle_margin and abs(r - d_lidar) <= self.distance_margin:
                matches.append((beam_angle, r))

        if not matches:
            if self.locked:
                # Still locked but no match yet, check timeout
                if (now - self.last_match_time).to_sec() > self.match_timeout:
                    rospy.logwarn("Lost LiDAR lock after %.2f s without matches, reverting to estimated angle", 
                                  self.match_timeout)
                else:
                    rospy.logdebug("Waiting for LiDAR matches at locked angle=%.2f deg", math.degrees(target_theta))
            else:
                rospy.logwarn("No LiDAR matches for camera angle=%.2f deg; lidar angle=%.2f deg", math.degrees(theta_cam), math.degrees(theta_lidar))
            return

        # Compute average range and average angle
        sum_r     = sum(r for (_, r) in matches)
        sum_theta = sum(theta for (theta, _) in matches)
        avg_r     = sum_r / len(matches)
        avg_theta = sum_theta / len(matches)

        # Lock onto this LiDAR measurement
        self.locked = True
        self.locked_r = avg_r
        self.locked_theta = avg_theta
        self.last_match_time = now

        # Compute horizontal and vertical components in LiDAR frame
        r_h = avg_r * math.sin(avg_theta)                          # meter
        r_v = avg_r * math.cos(avg_theta)                          # meter

        # Publish averaged ranges
        tdistance_msg = Float32MultiArrayStamped()
        tdistance_msg.header.stamp = rospy.Time.now()
        td_arr = Float32MultiArray()
        td_arr.data = [r_h, r_v]
        tdistance_msg.data = td_arr
        self.pub.publish(tdistance_msg)


        rospy.loginfo("Matched and locked: #matches=%d, range=%.2f m, angle=%.2f deg, horiz=%.2f m, vert=%.2f m", 
                      len(matches), avg_r, math.degrees(avg_theta), r_h, r_v)

if __name__ == '__main__':
    try:
        TrafPerceptionNode()
    except rospy.ROSInterruptException:
        pass







