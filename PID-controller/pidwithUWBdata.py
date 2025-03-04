import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    """ Generic PID controller function """
    error = setpoint - pv 
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = (kp * error + ki * integral + kd * derivative)
    return control, error, integral

class LaneControl:
    def __init__(self):
        rospy.init_node('lane_control', anonymous=True)
        
        # Subscribers for tag distances
        self.offcentre_distance_sub = rospy.Subscriber('/lane_following/offcentre_distance', Float64, self.offcentre_callback)
        self.orientation_angle_sub = rospy.Subscriber('/lane_following/orientation', Float64, self.orientation_angle_callback)
        
        # Publisher for vehicle commands
        self.to_vesc = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        
        # Error values
        self.offcentre_distance = None
        self.orientation_angle = None
        
        # PID parameters
        self.kp_offcentre, self.ki_offcentre, self.kd_offcentre = 1.5, 0.1, 0.05  # PID for off-center correction
        self.kp_orient, self.ki_orient, self.kd_orient = 2, 0.05, 0.03  # PID for orientation correction
        
        # PID states
        self.previous_error_offcentre = 0
        self.integral_offcentre = 0
        self.previous_error_orient = 0
        self.integral_orient = 0
        
        self.dt = 0.05
        self.rate = rospy.Rate(1 / self.dt)
    
    def offcentre_callback(self, msg): 
        self.offcentre_distance = msg.data
    
    def orientation_angle_callback(self, msg):
        self.orientation_angle = msg.data
    
    def compute_control(self):
        while not rospy.is_shutdown():
            if self.offcentre_distance is None or self.orientation_angle is None:
                continue
            
            # PID for off-center correction: generates a desired orientation angle
            desired_orientation, self.previous_error_offcentre, self.integral_offcentre = pid_controller(
                setpoint=0,
                pv=self.offcentre_distance,
                kp=self.kp_offcentre,
                ki=self.ki_offcentre,
                kd=self.kd_offcentre,
                previous_error=self.previous_error_offcentre,
                integral=self.integral_offcentre,
                dt=self.dt
            )
            
            # PID for orientation correction: aligns with the desired orientation
            orientation_correction, self.previous_error_orient, self.integral_orient = pid_controller(
                setpoint=desired_orientation,  # Now tracking the desired orientation, not just zero
                pv=self.orientation_angle,
                kp=self.kp_orient,
                ki=self.ki_orient,
                kd=self.kd_orient,
                previous_error=self.previous_error_orient,
                integral=self.integral_orient,
                dt=self.dt
            )
            
            # Steering is now only correcting orientation based on the desired angle
            steering_angle = orientation_correction
            
            # Set a constant speed for now
            speed = 10  # Adjust as necessary
            
            self.drive(speed, -steering_angle)  # Negative to correct steering direction
            self.rate.sleep()
    
    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.to_vesc.publish(msg)
        
if __name__ == '__main__':
    try:
        controller = LaneControl()
        controller.compute_control()
    except rospy.ROSInterruptException:
        pass
