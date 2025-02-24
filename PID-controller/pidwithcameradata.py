import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class LanePIDControl:
    def __init__(self):
        rospy.init_node('lane_pid_control', anonymous=True)

        # Subscribers for left and right lane errors
        self.x_error_left_sub = rospy.Subscriber('/lane_following/x_error_left', Float64, self.x_left_callback)
        self.phi_error_left_sub = rospy.Subscriber('/lane_following/phi_error_left', Float64, self.phi_left_callback)
        self.x_error_right_sub = rospy.Subscriber('/lane_following/x_error_right', Float64, self.x_right_callback)
        self.phi_error_right_sub = rospy.Subscriber('/lane_following/phi_error_right', Float64, self.phi_right_callback)
        
        # Publisher for vehicle control
        self.to_vesc = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)

        # Error variables
        self.x_error_left = None # assuming this is the data stream we will be getting, change values if necessasry
        self.phi_error_left = None
        self.x_error_right = None
        self.phi_error_right = None

        # PID parameters change these values as needed
        self.kp_x = 0.45
        self.ki_x = 0.0075
        self.kd_x = 0.001
        self.kp_phi = 0.45
        self.ki_phi = 0.0075
        self.kd_phi = 0.001

        self.integral_x = 0
        self.integral_phi = 0
        self.prev_error_x = 0
        self.prev_error_phi = 0
        
        self.prev_time = time.time()
        self.rate = rospy.Rate(50)

    def x_left_callback(self, msg):
        self.x_error_left = msg.data
    
    def phi_left_callback(self, msg):
        self.phi_error_left = msg.data
    
    def x_right_callback(self, msg):
        self.x_error_right = msg.data
    
    def phi_right_callback(self, msg):
        self.phi_error_right = msg.data

    def pid_controller(self, setpoint, pv, kp, ki, kd, integral, prev_error, dt):
        error = setpoint - pv
        integral += error * dt
        derivative = (error - prev_error) / dt
        control = kp * error + ki * integral + kd * derivative
        return control, error, integral
    
    def compute_control(self):
        curr_time = time.time()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # Handle missing data by averaging available values
        if self.x_error_left is not None and self.x_error_right is not None:
            x_error = (self.x_error_left + self.x_error_right) / 2    # for now but we can make changes depending on what ALI says
        elif self.x_error_left is not None:
            x_error = self.x_error_left
        elif self.x_error_right is not None:
            x_error = self.x_error_right
        else:
            x_error = 0

        if self.phi_error_left is not None and self.phi_error_right is not None:
            phi_error = (self.phi_error_left + self.phi_error_right) / 2
        elif self.phi_error_left is not None:
            phi_error = self.phi_error_left
        elif self.phi_error_right is not None:
            phi_error = self.phi_error_right
        else:
            phi_error = 0
        
        # PID control for lateral position
        steer_correction, self.prev_error_x, self.integral_x = self.pid_controller(
            0, x_error, self.kp_x, self.ki_x, self.kd_x, self.integral_x, self.prev_error_x, dt) # we can play around with these values. Change setpoint value as necessary
        
        # PID control for orientation
        orientation_correction, self.prev_error_phi, self.integral_phi = self.pid_controller(
            0, phi_error, self.kp_phi, self.ki_phi, self.kd_phi, self.integral_phi, self.prev_error_phi, dt) # we can play around with these values.Change setpoint value as necessary
        
        steering_angle = - (steer_correction + orientation_correction) # I am trying something crazy here lets see
        speed = 1.5  # Fixed speed for now we can change this if necessary
        
        self.send_drive_command(speed, steering_angle)

    def send_drive_command(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.to_vesc.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            self.compute_control()
            self.rate.sleep()

if __name__ == '__main__': #standard based on earlier code
    try:
        controller = LanePIDControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
