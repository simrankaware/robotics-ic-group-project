from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

try:
    begin = time.time()
    print(begin)
    while time.time() - begin < 5:
        time.sleep(1)
        print(time.time() - begin)
        try:
            target = BP.set_motor_power(BP.PORT_C, -45)
        except IOError as error:
            print(error)
    BP.set_motor_position(BP.PORT_B, target)
    begin = time.time()
    while time.time() - begin < 15:
        time.sleep(1)
        print(time.time() - begin)
    BP.reset_all()
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the >


