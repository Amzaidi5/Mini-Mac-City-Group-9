import time
import matplotlib.pyplot as plt
import numpy as np

def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    """ Generic PID controller function """
    error = setpoint - pv 
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = (kp * error + ki * integral + kd * derivative)
    return control, error, integral

def main():
    # Desired targets
    setpoint_position = 1  # Target position (meters)
    setpoint_orientation = 1  # Target orientation (angle in degrees)

    # Initial conditions
    pv_position = -40  # Initial position (meters)
    pv_orientation = 40  # Initial orientation (degrees)
    
    dt = 0.05  # Time step

    # Manually tuned PID parameters for Speed and Steering
    Kp_speed = 3
    Ki_speed = 0.1
    Kd_speed = 0.05

    Kp_steer = 3
    Ki_steer = 0.1
    Kd_steer = 0.05

    print(f"Manual PID Parameters for Speed Control: Kp={Kp_speed}, Ki={Ki_speed}, Kd={Kd_speed}")
    print(f"Manual PID Parameters for Steering Control: Kp={Kp_steer}, Ki={Ki_steer}, Kd={Kd_steer}")

    previous_error_pos = 0
    previous_error_ori = 0
    integral_pos = 0
    integral_ori = 0

    # Data storage for plotting
    time_steps = []
    position_values = []
    orientation_values = []
    speed_values = []
    steering_angle_values = []
    setpoint_pos_values = []
    setpoint_ori_values = []

    for i in range(100):  # Simulate for 100 time steps
        # PID for Speed (based on Position Error)
        speed, error_pos, integral_pos = pid_controller(setpoint_position, pv_position,
                                                        Kp_speed, Ki_speed, Kd_speed,
                                                        previous_error_pos, integral_pos, dt)
        
        # PID for Steering Angle (based on Orientation Error)
        steering_angle, error_ori, integral_ori = pid_controller(setpoint_orientation, pv_orientation,
                                                                 Kp_steer, Ki_steer, Kd_steer,
                                                                 previous_error_ori, integral_ori, dt)
        
        # Update position and orientation
        pv_position += speed * dt  # Position changes based on speed
        pv_orientation += steering_angle * dt  # Orientation changes based on steering

        # Store previous errors for next iteration
        previous_error_pos = error_pos
        previous_error_ori = error_ori

        # Store values for plotting
        time_steps.append(i * dt)
        position_values.append(pv_position)
        orientation_values.append(pv_orientation)
        speed_values.append(speed)
        steering_angle_values.append(steering_angle)
        setpoint_pos_values.append(setpoint_position)
        setpoint_ori_values.append(setpoint_orientation)

        time.sleep(dt)

    # Plot results
    plt.figure(figsize=(12, 10))

    # Position vs. Setpoint
    plt.subplot(3, 1, 1)
    plt.plot(time_steps, position_values, label='Position (m)')
    plt.plot(time_steps, setpoint_pos_values, label='Target Position', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Car Position Control')
    plt.legend()

    # Orientation vs. Setpoint
    plt.subplot(3, 1, 2)
    plt.plot(time_steps, orientation_values, label='Orientation (degrees)')
    plt.plot(time_steps, setpoint_ori_values, label='Target Orientation', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation (degrees)')
    plt.title('Car Orientation Control')
    plt.legend()

    # Speed and Steering Angle
    plt.subplot(3, 1, 3)
    plt.plot(time_steps, speed_values, label='Speed (m/s)')
    plt.plot(time_steps, steering_angle_values, label='Steering Angle (degrees)')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Outputs')
    plt.title('Speed and Steering Angle Over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
