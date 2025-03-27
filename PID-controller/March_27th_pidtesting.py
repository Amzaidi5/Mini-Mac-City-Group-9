#!/usr/bin/env python

import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt,flag):
    """ Generic PID controller function """

    error = setpoint - pv 
    integral = error * dt
    derivative = (error - previous_error) / dt
    control = (error)*(kp + ki * integral + kd * derivative)
    if flag ==0:
        print("output for original pid angle = ",control, error,integral)
        control=max(min(control, 0.3),-0.3)
        print("output for steering angle = ",control, error,integral)
    else:
        print("output for offcentre d = ",control, error,integral)
    return control, error, integral

class LaneControl:
    def __init__(self):
        rospy.init_node('lane_control', anonymous=True)
        
        # Subscribers for tag distances
        self.offcentre_distance_sub = rospy.Subscriber('/x_error_left', Float64, self.offcentre_left_callback)
        self.offcentre_distance_sub = rospy.Subscriber('/x_error_right', Float64, self.offcentre_right_callback)

        self.orientation_angle_sub = rospy.Subscriber('/orient_error', Float64, self.orientation_angle_callback)
        
        # Publisher for vehicle commands
        self.to_vesc = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        
        # Initialize error values (avoid None issues)
        self.offcentre_distance = 0.0
        self.orientation_angle = 0.0
        self.offcentre_left_distance = 0.0
        self.offcentre_right_distance = 0.0
        
        # PID parameters
        self.kp_offcentre, self.ki_offcentre, self.kd_offcentre = 1.5, 0.1, 0.05  # PID for off-center correction
        self.kp_orient, self.ki_orient, self.kd_orient = 2, 0.05, 0.03  # PID for orientation correction
        
        # PID states
        self.previous_error_offcentre = 0
        self.integral_offcentre = 0
        self.previous_error_orient = 0
        self.integral_orient = 0
        self.previous_steering = 0  # For rate-limiting steering changes

        # Derivative smoothing
        self.previous_derivative_offcentre = 0
        self.previous_derivative_orient = 0
        
        self.dt = 0.05
        self.rate = rospy.Rate(1 / self.dt)
    def offcentre_callback(self, msg): 
        self.offcentre_distance = msg.data
    def offcentre_left_callback(self, msg): 
        self.offcentre_left_distance = msg.data
    def offcentre_right_callback(self, msg): 
        self.offcentre_right_distance = msg.data
    def orientation_angle_callback(self, msg):
        self.orientation_angle = msg.data
    
    def compute_control(self):
        while not rospy.is_shutdown():
            if self.offcentre_left_distance is None and self.offcentre_right_distance is not None:
                self.offcentre_distance=self.offcentre_right_distance
            elif self.offcentre_right_distance is None and self.offcentre_left_distance is not None:
                self.offcentre_distance=self.offcentre_left_distance
            elif self.offcentre_left_distance is None and self.offcentre_right_distance is None :
                self.drive(0, 0) 
                rospy.sleep(1)     
            elif abs(self.offcentre_left_distance- self.offcentre_right_distance)>10 :
                self.drive(0, 0) 
                rospy.sleep(1)
            else:
                self.offcentre_distance=(self.offcentre_left_distance+self.offcentre_right_distance)/2                    


            if self.offcentre_distance is None or self.orientation_angle is None:
                rospy.sleep(0.1)  # Allow time for callbacks to update values
                continue  # Skip this iteration until values are updated
            if self.offcentre_distance <=5 and self.offcentre_distance>=-5:
                self.offcentre_distance =0

            # PID for off-center correction: generates a desired orientation angle
            desired_orientation, self.previous_error_offcentre, self.integral_offcentre = pid_controller(
                setpoint=0,
                pv=self.offcentre_distance,
                kp=self.kp_offcentre,
                ki=self.ki_offcentre,
                kd=self.kd_offcentre,
                previous_error=self.previous_error_offcentre,
                integral=self.integral_offcentre,
                dt=self.dt,
                flag=1
            )

            # Smooth the derivative term (to avoid sudden jumps)
            alpha = 0.8  # Smoothing factor
            derivative_offcentre = alpha * (desired_orientation - self.previous_derivative_offcentre) / self.dt + (1 - alpha) * self.previous_derivative_offcentre
            self.previous_derivative_offcentre = derivative_offcentre

            # PID for orientation correction: aligns with the desired orientation
            orientation_correction, self.previous_error_orient, self.integral_orient = pid_controller(
                setpoint=desired_orientation,  # Tracking desired orientation
                pv=self.orientation_angle,
                kp=self.kp_orient,
                ki=self.ki_orient,
                kd=self.kd_orient,
                previous_error=self.previous_error_orient,
                integral=self.integral_orient,
                dt=self.dt,
                flag=0
            )

            # Smooth steering changes (rate-limiting)
            #max_steering_rate = 0.1  # Maximum steering angle change per iteration
            steering_angle = max(min(orientation_correction, 0.3),-0.3)
            self.previous_steering = steering_angle

            # Set a constant speed (could be dynamically adjusted based on conditions)
            speed = 1  # Adjust as necessary

            self.drive(speed, -steering_angle)  # Negative to correct steering direction
            print("steering angle = ",-steering_angle)
            rospy.sleep(.2)  # Allow time for ROS to process other callbacks
            self.rate.sleep()
    
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.to_vesc.publish(msg)

    def shutdown(self):
        """ Emergency stop function when ROS is shutting down """
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
