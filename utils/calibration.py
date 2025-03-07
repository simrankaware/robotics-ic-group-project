from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C

#Parameters
BASE_POWER = 25
kp = 1.5

def calibrate_straight_line(base_power, duration):
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    start_time = time.time()
    print(start_time)

    total_l = 0
    total_r = 0
    count = 0

    while time.time() - start_time < duration:
        left_encoder = BP.get_motor_encoder(LEFT_MOTOR)
        right_encoder = BP.get_motor_encoder(RIGHT_MOTOR)

        # calculating error diff
        error = left_encoder - right_encoder
        print("Error: %6d" % error)
        # calculating correction 
        correction = error * kp

        left_power = base_power - correction
        right_power = base_power + correction

        total_l += left_power
        total_r += right_power
        count += 1
        
        BP.set_motor_power(LEFT_MOTOR, left_power)
        BP.set_motor_power(RIGHT_MOTOR, right_power)

        print("Left encoder: %6d  Right encoder: %6d" % (left_encoder, right_encoder))

        time.sleep(0.05)
    
    print("Average left power: ", total_l/count)
    print("Average right power: ", total_r/count)
    return total_l/count, total_r/count

if __name__ == "__main__":
    l, r = calibrate_straight_line(BASE_POWER, 20)
    print("\nCalibration done\n")
    print("Left: ", l)
    print("Right: ", r)