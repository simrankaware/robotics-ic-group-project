from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C

#Parameters
BASE_POWER = -15
kp = 1.5

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



def rotate(degrees):

    # degrees = (degrees / 90) * ANGLE_CALIBRATION
    
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    # Assuming 360 degrees corresponds to a full rotation of the motor
    target_degrees = degrees * (360 / 90)  # Adjust this factor based on your robot's configuration
    _, _, _, position_l = BP.get_motor_encoder(LEFT_MOTOR)
    _, _, _, position_r = BP.get_motor_encoder(RIGHT_MOTOR)
    BP.set_motor_position(LEFT_MOTOR, position_l-225)
    BP.set_motor_position(RIGHT_MOTOR, position_r +225)
    print( "Position L: ", position_l, "Position R: ", position_r)
    # while True:
    #     left_encoder = BP.get_motor_encoder(LEFT_MOTOR)
    #     right_encoder = BP.get_motor_encoder(RIGHT_MOTOR)

    #     # if abs(left_encoder) >= abs(target_degrees) and abs(right_encoder) >= abs(target_degrees):
    #     #     break

    #     time.sleep(0.05)

    BP.set_motor_power(LEFT_MOTOR, 0)
    BP.set_motor_power(RIGHT_MOTOR, 0)



try:
    move_in_straight_line(5)
    rotate(ANGLE_CALIBRATION)
    
    
 
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the >


