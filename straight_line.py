from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from math import pi
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C
WHEEL_RADIUS = 3.3
AXLE_RADIUS = 6.5
WHEEL_ROTATION_FOR_90_DEGREES = ((AXLE_RADIUS * pi) / 2) / (2 * pi * WHEEL_RADIUS) * 360

#Parameters
BASE_POWER = -15
kp = 1.5

TIME_DELAY = 300 / (120 * pi) 

ANGLE_CALIBRATION = 39.75


def move_in_straight_line(duration):
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    start_time = time.time()
    print(start_time)

    while time.time() - start_time < duration:
        left_encoder = BP.get_motor_encoder(LEFT_MOTOR)
        right_encoder = BP.get_motor_encoder(RIGHT_MOTOR)

        # calculating error diff
        error = left_encoder - right_encoder
        print("Error: %6d" % error)
        # calculating correction 
        correction = error * kp

        left_power = BASE_POWER - correction
        right_power = BASE_POWER + correction

        BP.set_motor_power(LEFT_MOTOR, left_power)
        BP.set_motor_power(RIGHT_MOTOR, right_power)

        print("Left encoder: %6d  Right encoder: %6d" % (left_encoder, right_encoder))

        time.sleep(0.05) 

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


def rotate(degrees, speed=25):  # Add a speed parameter (default: 50 dps)
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


# def rotate(degrees):

#     # degrees = (degrees / 90) * ANGLE_CALIBRATION
    
#     BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
#     BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

#     # Assuming 360 degrees corresponds to a full rotation of the motor
#     target_degrees = degrees * (360 / 90)  # Adjust this factor based on your robot's configuration
#     position_l = BP.get_motor_encoder(LEFT_MOTOR)
#     position_r = BP.get_motor_encoder(RIGHT_MOTOR)
#     BP.set_motor_dps(LEFT_MOTOR, position_l+155)
#     BP.set_motor_position(RIGHT_MOTOR, position_r-155)
#     # print( "Position L: ", position_l, "Position R: ", position_r)
#     # while True:
#     #     left_encoder = BP.get_motor_encoder(LEFT_MOTOR)
#     #     right_encoder = BP.get_motor_encoder(RIGHT_MOTOR)

#     #     if ((left_encoder) <= (position_l-225)) or ((right_encoder) >= (position_r+225)):
#     #         break

#     time.sleep(3)

#     BP.set_motor_power(LEFT_MOTOR, 0)
#     BP.set_motor_power(RIGHT_MOTOR, 0)



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


