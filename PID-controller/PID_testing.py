#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import time

class LaneControl:
    def __init__(self):
        rospy.init_node('lane_control', anonymous=True)

        # Subscribers
        rospy.Subscriber('/lane_following/x_error_left', Float64, self.x_left_callback)
        #rospy.Subscriber('/lane_following/x_error_right', Float64, self.x_right_callback)
        rospy.Subscriber('/vesc/commands/motor/speed', Float64, self.speed_callback)

        # Publisher
        self.pub = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)

        # Error Variables
        self.x_error_left = self.x_error_right = -1
        self.prev_x_error = 0
        self.x_error_accum = 0

        # Drive Parameters
        self.speed = 1.5
        self.angle = 0.0
        self.prev_time = time.time()

        self.rate = rospy.Rate(50)  # 50 Hz
        self.control_loop()

    def x_left_callback(self, msg):
        self.x_error_left = msg.data

    def x_right_callback(self, msg):
        self.x_error_right = msg.data

    def speed_callback(self, msg):
        self.speed = max(1.5, msg.data / 4614.0)  # Keep minimum speed

    def compute_control(self):
        curr_time = time.time()
        time_delta = curr_time - self.prev_time
        self.prev_time = curr_time

        kp_x, kd_x, ki_x = 0.45, 0.001, 0.0075

        x_error = self.x_error_left if self.x_error_right == -1 else (self.x_error_left + self.x_error_right) / 2

        x_p = kp_x * (x_error - 540) / 540.0
        x_d = kd_x * ((x_error - self.prev_x_error) / time_delta)
        self.x_error_accum += (x_error / 540.0) * time_delta

        self.angle = -(x_p + x_d + ki_x * self.x_error_accum)
        self.prev_x_error = x_error

    def drive(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.angle
        self.pub.publish(msg)

    def control_loop(self):
        try:
            while not rospy.is_shutdown():
                self.compute_control()
                self.drive()
                self.rate.sleep()
        except KeyboardInterrupt:
            self.speed = 0
            self.angle = 0
            self.drive()
            rospy.loginfo("Stopping")

if __name__ == '__main__':
    try:
        LaneControl()
    except rospy.ROSInterruptException:
        pass
