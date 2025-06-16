#!/usr/bin/env python

import math
from sensor_msgs.msg import LaserScan
import rospy
from std_msgs.msg import Float32, String, Float32MultiArray
from darknet_ros_msgs.msg import BoundingBoxes
import os
import time
from tf_controller_pkg.msg import Float32MultiArrayStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

class FSM(object):
    # ----- Enumerate the states-----
    STATE1, STATE2, STATE3 = range(1, 4)

    def __init__(self):
        # --- Reactive data ---
        self.distance = 0.0
        self.color = None
        self.car_distance = 0.0
        self.velocity = 0.0

        # --- State initialization ---
        self.state = self.STATE1
        self.throttle = 0.0
        self.stopping_distance = 0.40

        # --- Controller Constants ---
        self.kp_pos = 2.0
        self.kp_vel = 7.0

        # --- Subs and pubs ---
        self.color = Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
        self.distance = Subscriber('/traf_distances', Float32MultiArrayStamped)
        self.throttle_pub = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=10)
        self.rate = rospy.Rate(10)          # 10 Hz publish loop
        self.vel_sub = rospy.Subscriber('arduino_data_1', Float32MultiArray, self._velocity_callback, queue_size=1)

    # --- Time synchronization of subs ---
        self.ts = ApproximateTimeSynchronizer(
            [self.color, self.distance],
            queue_size=10,
            slop=0.1  # seconds
        )
        self.ts.registerCallback(self.callback)    

        self.car_distance = rospy.Subscriber('/scan', LaserScan, self._scan_callback, queue_size=1)

    def callback(self, boxes_msg, distance_msg):
        self.distance = distance_msg.data.data[1]
        best = max(boxes_msg.bounding_boxes, key=lambda b: b.probability)
        self.color = best.Class if best else None
        rospy.loginfo_throttle(0.5, self.distance)

    def _velocity_callback(self, array_msg):
        self.velocity = array_msg.data[3]
        rospy.loginfo_throttle(1.0, "[ARDUINO] velocity=%.2f", self.velocity)
        
    def _scan_callback(self, scan_msg):
    # beam closest to 0 rad
        idx = int(round((0.0 - scan_msg.angle_min) / scan_msg.angle_increment))
        idx = max(0, min(idx, len(scan_msg.ranges) - 1))

        r0 = scan_msg.ranges[idx]
        if math.isinf(r0) or math.isnan(r0) or r0 == 0.0:
            self.car_distance = None
        else:
            self.car_distance = r0

        if self.car_distance:
            rospy.loginfo_throttle(0.5, "[LiDAR] car d=%.2f", self.car_distance)

    def _update_state(self):
        if (rospy.Time.now() - self.last_dist_stamp > rospy.Duration(1.0)
                or self.color is None):
            return  
      
    def spin(self):
        while not rospy.is_shutdown():
            self._update_state()      # decide transitions + set throttle
            self._publish_throttle()  # push out the command
            self.rate.sleep()

    def _update_state(self):
        if self.distance is None or self.color is None:
            return                    # wait until both sensors have data

        # ---------------- STATE 1 : Driving ----------------
        if self.state == self.STATE1:
            rospy.loginfo_throttle(0.1, "State = 1")
            self.throttle = 1.0
            if self.color == 'Green':
                pass
            elif self.color == 'Yellow' and self.distance < 0.50:
                pass
            elif self.distance > 1.0 and self.color == 'Red':
                pass    
            elif self.car_distance is not None and (self.stopping_distance < self.car_distance < 1.5):
                self.state = self.STATE2
            elif self.distance > 0.50 and self.color == 'Yellow':
                self.state = self.STATE2
            elif self.stopping_distance < self.distance < 1.0 and self.color == 'Red':
                self.state = self.STATE2                                          
            elif self.car_distance < self.stopping_distance:
                self.state = self.STATE3
            elif self.color == 'Yellow' and self.distance < self.stopping_distance:
                self.state = self.STATE3
            elif self.color == 'Red' and self.distance < self.stopping_distance:
                self.state = self.STATE3
            else:
                pass

        # ---------------- STATE 2 : Slowing Down ----------------
        elif self.state == self.STATE2:
            rospy.loginfo_throttle(0.1, "State = 2")
            pos_error = self.distance - self.stopping_distance
            self.throttle = self.kp_vel * (self.kp_pos * pos_error - self.velocity)
            if self.car_distance is not None and self.stopping_distance < self.car_distance < 1.5:
                pass
            elif self.color == 'Yellow' and self.distance > self.stopping_distance:          
                pass
            elif self.color == 'Red' and self.stopping_distance < self.distance < 1.0:                         
                pass
            elif self.color == 'Green' and self.car_distance > 1.0:
                self.state = self.STATE1
            elif self.car_distance is not None and self.car_distance < self.stopping_distance:
                self.state = self.STATE3
            elif self.color == 'Red' and self.distance < self.stopping_distance:                         
                self.state = self.STATE3
            elif self.color == 'Yellow' and self.distance < self.stopping_distance:             
                self.state = self.STATE3
            else:                                                     
                pass

        # ---------------- STATE 3 : Stopping ----------------
        elif self.state == self.STATE3:
            rospy.loginfo_throttle(0.1, "State = 3")
            self.throttle = 0.0
            if self.car_distance is not None and self.car_distance < self.stopping_distance:
                pass
            elif self.color == 'Red':
                pass
            elif self.color == 'Green' and self.car_distance > 1.0:
                self.state = self.STATE1  
            elif (self.color == 'Yellow' or self.color == 'Red') and self.car_distance > 1.0:
                self.state = self.STATE2
            else:
                pass
            

    def _publish_throttle(self):
        self.throttle_pub.publish(Float32(self.throttle))


def main():
    rospy.init_node('fsm_node')
    FSM().spin()


if __name__ == '__main__':
    car_number = os.environ["car_number"]
    main()
