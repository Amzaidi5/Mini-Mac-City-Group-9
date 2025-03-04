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
        self.offcentre_distance_sub = rospy.Subscriber('/lane_following/offcentre_distance', Float64, self.offcentre_callback) # we need to set this data stream up essentially
        self.orientation_angle_sub = rospy.Subscriber('/lane_following/orientation', Float64, self.orientation_angle_callback) # we need to set this data stream up essentially
        
        # Publisher for vehicle commands
        self.to_vesc = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
        
        # Error values
        self.offcentre_distance = None # I think you guys have the math to test this out if I am not wrong.
        self.orientation = None
        
        
        # PID parameters
        self.kp_steer, self.ki_steer, self.kd_steer = 3, 0.1, 0.05 # we need to test to figure out these values, dummy values for now
        #self.kp_speed, self.ki_speed, self.kd_speed = 3, 0.1, 0.05 # we need to test to figure out these values, dummy values for now
        
        # PID states
        self.previous_error_steer = 0
        self.integral_steer = 0
        
        self.dt = 0.05
        self.rate = rospy.Rate(1 / self.dt) # we need to set this up essentially depending on how often uwb sends data to make it less error prone
        
    def offcentre_callback(self, msg): 
        self.offcentre_distance = msg.data
    
    def orientation_angle_callback(self, msg):
        self.orientation_angle = msg.data
    
    def compute_control(self):
        while not rospy.is_shutdown():
            if self.offcentre_distance is None:
                continue
            
            if (self.offcentre_distance<5 and self.offcentre_distance>5)  and self.orientation_angle==0:
                print("centred")
            
            # Compute steering control
            steering_angle, self.previous_error_steer, self.integral_steer = pid_controller( #figure out exact values later for setpoint
                setpoint=0,
                pv=self.offcentre_distance,
                kp=self.kp_steer,
                ki=self.ki_steer,
                kd=self.kd_steer,
                previous_error=self.previous_error_steer,
                integral=self.integral_steer,
                dt=self.dt
            )
            
            # Compute speed control based on distance from tag
            
            speed=10 #dummy value for now
            self.drive(speed, -steering_angle) #negative steering angle essentially flips how the car needs to move!! if theres an error just make the change beforehand before sending the value to the vesc.
            self.rate.sleep() # might be a buffer I am not sure.
    
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
