from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from math import pi, atan2, sqrt
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C
WHEEL_RADIUS = 3.05
AXLE_RADIUS = 6.5
WHEEL_ROTATION_FOR_1_DEGREE = ((AXLE_RADIUS * pi) / 2) / (2 * pi * WHEEL_RADIUS) * 4

# Parameters
BASE_POWER = -15
kp = 1.5

TIME_DELAY = 300 / (120 * pi) 
LINE_TIME_MULTIPLIER = 0.9
ROTATION_TIME_MULTIPLIER = 0.88
ANG_VELOCITY = 350
ANGLE_CALIBRATION = 39.75

def drive_straight_for_distance(distance, speed=ANG_VELOCITY): # distance in cm
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    rotations = distance / (WHEEL_RADIUS * pi * 2)
    print(f"Number of rotations per straight: f{rotations}")
    target_degrees = rotations * 360
    print(f"Degrees of rotation required per straight: f{target_degrees}")
    BP.set_motor_dps(LEFT_MOTOR, speed)   # Set slow speed
    BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation
    sleep_time = abs(target_degrees / speed)
    #sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
    sleep_time = sleep_time * LINE_TIME_MULTIPLIER
    time.sleep(sleep_time)  # Wait for rotation to complete
    print(f"Slept for f{sleep_time} seconds")
    print(f"Done with straight at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n R: {BP.get_motor_encoder(RIGHT_MOTOR)}")


def rotate(degrees, speed=ANG_VELOCITY):  # Add a speed parameter (default: 50 dps)
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    target_degrees = degrees # Adjust factor based on calibration

    BP.set_motor_dps(LEFT_MOTOR, -speed)   # Set slow speed
    BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation

    sleep_time = abs(target_degrees / speed)
    #sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
    sleep_time = sleep_time * ROTATION_TIME_MULTIPLIER
    time.sleep(sleep_time)  # Wait for rotation to complete
    print(f"Slept for f{sleep_time} seconds")
    print(f"Done with rotate at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n  R: {BP.get_motor_encoder(RIGHT_MOTOR)}")




def navigate_to_waypoint(destination, current_position):
    wx, wy = destination
    x, y, theta = current_position
    dx = wx - x
    dy = wy - y
    bearing = atan2(dy, dx)
    angle_to_rotate = (bearing - theta) * (180 / pi) # Convert to degrees for the rotate function
    distance = sqrt(dx**2 + dy**2)
    print(f"Rotating through angle: {angle_to_rotate}")
    rotate(angle_to_rotate * WHEEL_ROTATION_FOR_1_DEGREE)
    BP.set_motor_dps(LEFT_MOTOR, 0)
    BP.set_motor_dps(RIGHT_MOTOR, 0)
    time.sleep(1.5)
    print(f"Driving for distance: {distance}")
    drive_straight_for_distance(distance)
    BP.set_motor_dps(LEFT_MOTOR, 0)
    BP.set_motor_dps(RIGHT_MOTOR, 0)
    return (wx, wy, bearing)


if __name__ == "__main__":
    
    current_position = (0, 0, 0)
    
    #while True:
        #(destination_x, destination_y) = input("Enter the waypoint coordinates (x, y) in metres: ")
    destination_x, destination_y = 0.4, 0.4
    destination = (destination_x * 100, destination_y * 100)
    current_position = navigate_to_waypoint(destination, current_position)

