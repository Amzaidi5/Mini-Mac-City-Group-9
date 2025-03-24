#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    """ PID controller for off-center distance correction """
    error = setpoint - pv 
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = (kp * error + ki * integral + kd * derivative)
    return control, error, integral

class LaneControl:
    def __init__(self):
        rospy.init_node('lane_control_left_only', anonymous=True)
        
        # Subscriber for off-center distance from left camera
        self.offcentre_distance_sub = rospy.Subscriber('/lane_following/x_error_left', Float64, self.offcentre_callback)
        
        # Publisher for vehicle commands
        self.to_vesc = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        
        # Initialize error values
        self.offcentre_distance = 0.0
        
        # PID parameters for off-center correction (tuned for smoother control)
        self.kp, self.ki, self.kd = 1.2, 0.05, 0.08  # Adjusted values

        # PID states
        self.previous_error = 0
        self.integral = 0
        self.previous_steering = 0  # For rate-limiting steering changes
        
        self.dt = 0.05
        self.rate = rospy.Rate(1 / self.dt)

        # Derivative smoothing
        self.previous_derivative = 0
    
    def offcentre_callback(self, msg): 
        # Apply a simple low-pass filter to reduce noise
        alpha = 0.7  # Smoothing factor (tunable)
        self.offcentre_distance = alpha * self.offcentre_distance + (1 - alpha) * msg.data
    
    def compute_control(self):
        while not rospy.is_shutdown():
            # Compute PID control based on off-center distance
            raw_steering_angle, self.previous_error, self.integral = pid_controller(
                setpoint=0,  # Target is to be centered
                pv=self.offcentre_distance,
                kp=self.kp,
                ki=self.ki,
                kd=self.kd,
                previous_error=self.previous_error,
                integral=self.integral,
                dt=self.dt
            )

            # Apply derivative smoothing
            alpha = 0.8  # Smoothing factor
            derivative = alpha * (raw_steering_angle - self.previous_derivative) / self.dt + (1 - alpha) * self.previous_derivative
            self.previous_derivative = derivative
            
            # Scale & limit the steering output
            max_steering_angle = 0.3  # Maximum allowable steering angle
            steering_angle = max(min(raw_steering_angle, max_steering_angle), -max_steering_angle)

            # Smooth steering changes (rate-limiting)
            max_steering_rate = 0.05  # Adjust to prevent aggressive oscillations
            steering_angle = self.previous_steering + max(min(steering_angle - self.previous_steering, max_steering_rate),
                                                          -max_steering_rate)
            self.previous_steering = steering_angle

            # Set a constant speed
            speed = 1.5  # Adjust as necessary
            
            self.drive(speed, -steering_angle)  # Negative to correct steering direction
            
            rospy.sleep(0.01)  # Allow time for ROS to process other callbacks
            self.rate.sleep()
    
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.to_vesc.publish(msg)

    def shutdown(self):
        rospy.loginfo("Shutting down. Stopping the vehicle.")
        self.drive(0, 0)  # Stop the car
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        controller = LaneControl()
        rospy.on_shutdown(controller.shutdown)
        controller.compute_control()
    except rospy.ROSInterruptException:
        controller.shutdown()
