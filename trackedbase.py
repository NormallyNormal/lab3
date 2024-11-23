import time
import math
import mathutil

wheel_diameter_cm = 5.35
base_width = 9.4488
pid_timeout = 5
def wheel_degrees_to_cm(degrees):
    # Convert to radians, then convert to centimeters
    return degrees/180 * 3.1415 * wheel_diameter_cm/2

def differential_drive_motion_angular(u_l, u_r, angle, dt):
    # Call differential drive motion, with angular velocities converted to linear velocities and degrees converted to radians.
    return differential_drive_motion(wheel_degrees_to_cm(u_l), wheel_degrees_to_cm(u_r), angle/180 * 3.1415, dt)

def differential_drive_motion(v_l, v_r, theta, dt):
    # Velocity of center
    v = (v_r + v_l) / 2.0
    # Angular velocity of robot in radians
    omega = (v_r - v_l) / base_width
    delta_x, delta_y, delta_theta = 0, 0, 0
    # If moving straight
    if omega == 0:
        delta_x = v * dt
        delta_y = 0
    # If not moving straight
    else:
        # ICC radius
        radius = v / omega
        # Change of theta for this movement step
        delta_theta = omega * dt
        # Change of position for this movement step
        delta_x = radius * (math.sin(delta_theta))
        delta_y = radius * (1 - math.cos(delta_theta))
    # Rotate position update based on the actual angle of the robot
    adjusted_delta_x = delta_x * math.cos(theta) - delta_y * math.sin(theta)
    adjusted_delta_y = delta_x * math.sin(theta) + delta_y * math.cos(theta)
    return adjusted_delta_x, adjusted_delta_y, delta_theta

class TrackedBase:
    def __init__(self, left_motor, right_motor, gyroscope):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyroscope = gyroscope
        self.x = 0
        self.y = 0

        self.prev_time = time.time()
        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.prev_angle = gyroscope.angle()
    
    def drive(self, left_speed, right_speed):
        # Time since last drive call
        delta_time = time.time() - self.prev_time
        self.prev_time = time.time()

        # Change in position since last drive call
        delta_x, delta_y, delta_theta = 0, 0, 0
        if delta_time < 0.05:
            delta_x, delta_y, delta_theta = differential_drive_motion_angular(self.left_motor.speed(), self.right_motor.speed(), self.prev_angle, delta_time)
        else:
            delta_x, delta_y, delta_theta = differential_drive_motion_angular(self.prev_left_speed, self.prev_right_speed, self.prev_angle, delta_time)

        self.x += delta_x
        self.y += delta_y

        self.prev_left_speed = left_speed
        self.prev_right_speed = right_speed
        self.prev_angle = self.gyroscope.angle()

        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    def get_position(self):
        return self.x, self.y

    def reset_position(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def get_heading(self):
        return self.gyroscope.angle()

    def drive_distance(self, distance, speed):
        left_start_angle = self.left_motor.angle()
        right_start_angle = self.right_motor.angle()
        distance_traveled = 0
        slowzone = speed / 100
        speed = speed * mathutil.sign(distance)
        # Drive until the distance is reached
        start_time = time.time() 
        while abs(distance_traveled) < abs(distance) and time.time() - start_time < pid_timeout:
            delta_distance = abs(distance) - abs(distance_traveled)
            delta_distance = min(slowzone, delta_distance)
            delta_distance = max(1, delta_distance)
            p = float(delta_distance) / slowzone
            self.drive(speed * p, speed * p)
            distance_traveled = wheel_degrees_to_cm(((self.left_motor.angle() - left_start_angle) + (self.right_motor.angle() - right_start_angle)) / 2)
            time.sleep(0.01)
        self.drive(0, 0)
    
    def turn_angle(self, angle, speed):
        start_angle = self.get_heading()
        angle_traveled = 0
        speed = min(speed, 240)
        slowzone = speed / 10
        speed = speed * mathutil.sign(angle)
        start_time = time.time() 
        # Turn until the distance is reached
        while abs(angle_traveled) < abs(angle) and time.time() - start_time < pid_timeout:
            delta_angle = abs(angle) - abs(angle_traveled)
            delta_angle = min(slowzone, delta_angle)
            delta_angle = max(2, delta_angle)
            p = float(delta_angle) / slowzone
            self.drive(-speed * p, speed * p)
            angle_traveled = (self.get_heading() - start_angle)
            time.sleep(0.01)
        self.drive(0, 0)
    
    def turn_to_angle(self, target_angle, speed):
        current_angle = self.get_heading()
        
        # Calculate the shortest angle difference
        angle_difference = mathutil.looping_minimal_angle_difference(target_angle, current_angle)

        self.turn_angle(angle_difference, speed)
