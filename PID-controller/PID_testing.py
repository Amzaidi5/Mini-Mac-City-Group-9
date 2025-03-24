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
    
    rospy.loginfo("PID Output: control=%.4f, error=%.4f, integral=%.4f, derivative=%.4f", control, error, integral, derivative)
    
    return control, error, integral

class LaneControl:
    def __init__(self):
        rospy.init_node('lane_control_left_only', anonymous=True)
        
        self.offcentre_distance_sub = rospy.Subscriber('/x_error', Float64, self.offcentre_callback)
        self.to_vesc = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        
        self.offcentre_distance = 0.0
        self.kp, self.ki, self.kd = 1.2, 0.05, 0.08  
        
        self.previous_error = 0
        self.integral = 0
        self.previous_steering = 0  
        
        self.dt = 0.05
        self.rate = rospy.Rate(1 / self.dt)
        self.previous_derivative = 0
    
    def offcentre_callback(self, msg): 
        alpha = 0.6  # Adjusted smoothing factor
        self.offcentre_distance = alpha * self.offcentre_distance + (1 - alpha) * msg.data
    
    def compute_control(self):
        while not rospy.is_shutdown():
            if abs(self.offcentre_distance) <= 5:
                self.offcentre_distance *= 0.2  # Soft transition instead of abrupt zeroing
            
            raw_steering_angle, self.previous_error, self.integral = pid_controller(
                setpoint=0, 
                pv=self.offcentre_distance,
                kp=self.kp,
                ki=self.ki,
                kd=self.kd,
                previous_error=self.previous_error,
                integral=self.integral,
                dt=self.dt
            )
            rospy.loginfo("Off-center distance: %.4f", self.offcentre_distance)

            alpha = 0.7  # Adjusted derivative smoothing
            derivative = alpha * (raw_steering_angle - self.previous_derivative) / self.dt + (1 - alpha) * self.previous_derivative
            self.previous_derivative = derivative
            
            max_steering_angle = 0.3
            steering_angle = max(min(raw_steering_angle, max_steering_angle), -max_steering_angle)
            
            max_steering_rate = 0.08  # Adjusted for slightly faster response
            steering_angle = self.previous_steering + max(min(steering_angle - self.previous_steering, max_steering_rate),
                                                          -max_steering_rate)
            self.previous_steering = steering_angle
            
            rospy.loginfo("Steering Output: raw=%.4f, limited=%.4f", raw_steering_angle, steering_angle)
            
            speed = 1.5  
            self.drive(speed, -steering_angle)
            rospy.loginfo("Final Steering Command: %.4f", -steering_angle)
            
            rospy.sleep(0.01)
            self.rate.sleep()
    
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.to_vesc.publish(msg)

    def shutdown(self):
        rospy.loginfo("Shutting down. Stopping the vehicle.")
        for _ in range(5):  # Ensure stopping command is sent multiple times
            self.drive(0, 0)
            rospy.sleep(0.2)

if __name__ == '__main__':
    try:
        controller = LaneControl()
        rospy.on_shutdown(controller.shutdown)
        controller.compute_control()
    except rospy.ROSInterruptException:
        controller.shutdown()
