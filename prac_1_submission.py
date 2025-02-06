from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from math import pi
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C
WHEEL_RADIUS = 3.05
AXLE_RADIUS = 6.5
WHEEL_ROTATION_FOR_90_DEGREES = ((AXLE_RADIUS * pi) / 2) / (2 * pi * WHEEL_RADIUS) * 360

#Parameters
BASE_POWER = -15
kp = 1.5

TIME_DELAY = 300 / (120 * pi) 

ANGLE_CALIBRATION = 39.75


def drive_straight_for_distance(distance, speed=-20): # distance in cm
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    rotations = distance / (WHEEL_RADIUS * pi * 2)
    print(f"Number of rotations per straight: f{rotations}")
    target_degrees = rotations * 360
    print(f"Degrees of rotation required per straight: f{target_degrees}")
    BP.set_motor_dps(LEFT_MOTOR, speed)   # Set slow speed
    BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation
    sleep_time = abs(target_degrees / speed)
    sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
    time.sleep(sleep_time)  # Wait for rotation to complete
    print(f"Slept for f{sleep_time} seconds")
    print(f"Done with straight at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n R: {BP.get_motor_encoder(RIGHT_MOTOR)}")


def rotate(degrees, speed=35):  # Add a speed parameter (default: 50 dps)
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    target_degrees = degrees # Adjust factor based on calibration

    BP.set_motor_dps(LEFT_MOTOR, speed)   # Set slow speed
    BP.set_motor_dps(RIGHT_MOTOR, -speed) # Opposite direction for rotation

    sleep_time = abs(target_degrees / speed)
    sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
    time.sleep(sleep_time)  # Wait for rotation to complete
    print(f"Slept for f{sleep_time} seconds")
    print(f"Done with rotate at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n  R: {BP.get_motor_encoder(RIGHT_MOTOR)}")


try:
    drive_straight_for_distance(40, -120)
    rotate(WHEEL_ROTATION_FOR_90_DEGREES)
    drive_straight_for_distance(40, -120)
    rotate(WHEEL_ROTATION_FOR_90_DEGREES)
    drive_straight_for_distance(40, -120)
    rotate(WHEEL_ROTATION_FOR_90_DEGREES)
    drive_straight_for_distance(40, -120)
    rotate(WHEEL_ROTATION_FOR_90_DEGREES)
    BP.reset_all()
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the >


