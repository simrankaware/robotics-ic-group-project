from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C

#Parameters
BASE_POWER = 25

def move_in_straight_line(duration):
    try:
        begin = time.time()
        print(begin)
        while time.time() - begin < duration:
            time.sleep(1)
            print(time.time() - begin)
            try:
                BP.set_motor_power(LEFT_MOTOR, BASE_POWER)
                BP.set_motor_power(RIGHT_MOTOR, BASE_POWER)
            except IOError as error:
                print(error)
        BP.set_motor_power(LEFT_MOTOR, 0)
        BP.set_motor_power(RIGHT_MOTOR, 0)
        BP.reset_all()
    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all() 



def rotate(degrees):
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    # Assuming 360 degrees corresponds to a full rotation of the motor
    target_degrees = degrees * (360 / 90)  # Adjust this factor based on your robot's configuration

    BP.set_motor_position(LEFT_MOTOR, -target_degrees)
    BP.set_motor_position(RIGHT_MOTOR, target_degrees)

    while True:
        left_encoder = BP.get_motor_encoder(LEFT_MOTOR)
        right_encoder = BP.get_motor_encoder(RIGHT_MOTOR)

        if abs(left_encoder) >= abs(target_degrees) and abs(right_encoder) >= abs(target_degrees):
            break

        # Reduce power as it rotates
        BP.set_motor_power(LEFT_MOTOR, BASE_POWER / 2)
        BP.set_motor_power(RIGHT_MOTOR, BASE_POWER / 2)

        time.sleep(0.05)

    BP.set_motor_power(LEFT_MOTOR, 0)
    BP.set_motor_power(RIGHT_MOTOR, 0)



try:
    rotate(39.75)
    time.sleep(1)
    move_in_straight_line(5)
    rotate(39.75)
    time.sleep(1)
    time.sleep(1)
    move_in_straight_line(5)
    rotate(39.75)
    time.sleep(1)
    time.sleep(1)
    move_in_straight_line(5)
    rotate(39.75)
    move_in_straight_line(2)
    
 
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the >


