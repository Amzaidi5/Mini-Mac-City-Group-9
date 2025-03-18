#!/usr/bin/env python

import numpy as np
#import pandas as pd
import cv2

import rospy
import tf
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64


class orientation_get:
    def __init__(self):

	self.orient_input = rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.run_odom)
	self.orient_error = rospy.Publisher('orient_error', Float64, queue_size=3)

    def odom_callback(self, msg):
	print("Entered odom_callback")	
	# Extract IMU data
        #orientation = msg.orientation  # Quaternion (x, y, z, w)
        #angular_velocity = msg.angular_velocity  # Angular velocity (rad/s)
        #linear_acceleration = msg.linear_acceleration  # Acceleration (m/s^2)

	#orientation = msg.pose.pose.orientation
	#euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
	#roll = euler[0]
	#pitch = euler[1]
	#yaw = euler[2]
	
	quat = msg.pose.pose.orientation 
	x = quat.x
	y = quat.y
	z = quat.z
	w = quat.w

	roll, pitch, yaw = euler_from_quaternion([x, y, z, w])

	#roll = math.degrees(roll)
	#pitch = math.degrees(pitch)
	#yaw = math.degrees(yaw)
        
	return yaw

    def run_odom(self, msg):

        orientation = self.odom_callback(msg)
       	self.orient_error.publish(orientation)
       	print("Orientation: %f", orientation)

       	return orientation





def main(args=None):
    rospy.init_node('zed_imu_subscriber', anonymous=True)
    orient = orientation_get()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("stopping")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
