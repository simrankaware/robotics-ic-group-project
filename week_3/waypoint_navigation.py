#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from math import pi, atan2, sqrt, cos, sin, sqrt
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import random

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C
WHEEL_RADIUS = 3.05
AXLE_RADIUS = 8.5
WHEEL_ROTATION_FOR_1_DEGREE = ((AXLE_RADIUS * pi) / 2) / (2 * pi * WHEEL_RADIUS) * 4

# Parameters
BASE_POWER = -15
kp = 1.5

TIME_DELAY = 300 / (120 * pi)
LINE_TIME_MULTIPLIER = 0.9
ROTATION_TIME_MULTIPLIER = 0.88
ANG_VELOCITY = 350
ANGLE_CALIBRATION = 39.75



DRAW_SCALE = 10
X_OFFSET = 200
Y_OFFSET = 500

            
def sample_gaussian(mean=0, std_dev=1):
        return random.gauss(mean, std_dev)
# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin      = 0.05*map_size
        self.scale       = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self,line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data]
        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = []

    def add_wall(self,wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)
            


# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 100
        
        self.sd_e = 0.7  # s.d. for straight line
        self.sd_f = 0.001  # s.d. for rotation during straight line motion
        self.sd_g = 0.05 # s.d. for rotation
        
        
        self.data = [(0, 0, 0, 1/self.n) for _ in range(self.n)]

        
    def perturb_particle_straight_line(self, dist):
        for i in range(len(self.data)):
            (x, y, theta, w) = self.data[i]
            noise_xy = sample_gaussian(std_dev=self.sd_e)
            noise_theta = sample_gaussian(std_dev=self.sd_f)
            self.data[i] = (  ( x + (dist+noise_xy)*cos(theta)),
                              ( y + (dist+noise_xy)*sin(theta)),
                              theta + noise_theta,
                              w)
    
    def perturb_particle_rotation(self, angle):
        for i in range(len(self.data)):
            (x, y, theta, w) = self.data[i]
            self.data[i] = (x,
                            y,
                            theta + angle + sample_gaussian(std_dev=self.sd_g),
                            w)
    
    def get_point_estimate(self):
        x = 0
        y = 0
        theta = 0
        # weighted mean
        for (x_p, y_p, theta_p, w) in self.data:
            x += x_p * w
            y += y_p * w
            theta += theta_p * w
        return (x, y, theta)

    
    def draw(self):
        canvas.drawParticles(self.data)

        
        
class Robot:
    def __init__(self):
        self.position = (0, 0, 0)
        
    def update_robot_position_straight_line(self, dist):
        (x, y, theta) = self.position
        x += dist * cos(theta)
        y += dist * sin(theta)
        self.position = (x, y, theta)

    def update_robot_position_rotation(self, angle):
        (x, y, theta) = self.position
        theta += angle
        self.position = (x, y, theta)
    
    
    
    def drive_straight_for_distance(self, distance, speed=ANG_VELOCITY): # distance in cm
        BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
        BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

        rotations = distance / (WHEEL_RADIUS * pi * 2)
#        print(f"Number of rotations per straight: f{rotations}")
        target_degrees = rotations * 360
#        print(f"Degrees of rotation required per straight: f{target_degrees}")
        BP.set_motor_dps(LEFT_MOTOR, speed)   # Set slow speed
        BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation
        sleep_time = abs(target_degrees / speed)
        #sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
        sleep_time = sleep_time * LINE_TIME_MULTIPLIER
        time.sleep(sleep_time)  # Wait for rotation to complete
#        print(f"Slept for f{sleep_time} seconds")
#        print(f"Done with straight at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n R: {BP.get_motor_encoder(RIGHT_MOTOR)}")
#        self.update_robot_position_straight_line(distance)


    def rotate(self, degrees, speed=ANG_VELOCITY):  # Add a speed parameter (default: 50 dps)
        BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
        BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

#        print(f"rotated through {degrees} in rotate lolll!")
        target_degrees = degrees # Adjust factor based on calibration

        if degrees < 0:
            speed = -speed


        BP.set_motor_dps(LEFT_MOTOR, -speed)
        BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation

        sleep_time = abs(target_degrees / speed)
        #sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
        sleep_time = sleep_time * ROTATION_TIME_MULTIPLIER
        time.sleep(sleep_time)  # Wait for rotation to complete
#        print(f"Slept for f{sleep_time} seconds")
#        print(f"Done with rotate at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n  R: {BP.get_motor_encoder(RIGHT_MOTOR)}")
#        self.update_robot_position_rotation(degrees * (pi / 180))
            

    def navigate_to_waypoint(self, destination, particles):
        wx, wy = destination
        x, y, theta = self.position
        print(f"robot is currently at ({x}, {y}, {theta})")
        dx = wx - x
        dy = wy - y
        bearing = atan2(dy, dx)
        
        print(f"here is bearing: {bearing}, and here is theta {theta}")
        # Compute the difference and normalize it
        angle_to_rotate = (bearing - theta) * (180 / pi) # Convert to degrees for the rotate function
        
        print(f"Before putting into range, angle_to_rotate is: {angle_to_rotate}")
#        angle_to_rotate = (bearing - theta) * (180 / pi) # Convert to degrees for the rotate function
        # Normalize the angle to [-pi, pi] range
        if angle_to_rotate > 180:
            angle_to_rotate -= 360
        elif angle_to_rotate < -180:
            angle_to_rotate += 360
            
        distance = sqrt(dx**2 + dy**2)
        print(f"Rotating through angle: {angle_to_rotate} degrees")

        self.rotate(angle_to_rotate * WHEEL_ROTATION_FOR_1_DEGREE)

        # Pass the angle in radians to perturb_particle_rotation
        particles.perturb_particle_rotation(angle_to_rotate * pi/180)

        BP.set_motor_dps(LEFT_MOTOR, 0)
        BP.set_motor_dps(RIGHT_MOTOR, 0)
        time.sleep(1.5)
        print(f"Driving for distance: {distance}")
        self.drive_straight_for_distance(distance)

        particles.perturb_particle_straight_line(distance)
        
        old_position = self.position

        updated_position = particles.get_point_estimate()
        
        
        self.position = updated_position


        BP.set_motor_dps(LEFT_MOTOR, 0)
        BP.set_motor_dps(RIGHT_MOTOR, 0)
        return particles, (old_position, updated_position)

        

if __name__ == "__main__":
    
    canvas = Canvas()    # global canvas we are going to draw on

    mymap = Map()
    # Definitions of walls
    # a: O to A
    # b: A to B
    # c: C to D
    # d: D to E
    # e: E to F
    # f: F to G
    # g: G to H
    # h: H to O
    mymap.add_wall((0,0,0,168))        # a
    mymap.add_wall((0,168,84,168))     # b
    mymap.add_wall((84,126,84,210))    # c
    mymap.add_wall((84,210,168,210))   # d
    mymap.add_wall((168,210,168,84))   # e
    mymap.add_wall((168,84,210,84))    # f
    mymap.add_wall((210,84,210,0))     # g
    mymap.add_wall((210,0,0,0))        # h
    mymap.draw()

    particles = Particles()
    print(particles)
    robot = Robot()


    waypoints = [(0.5, 0.5), (0.5, 0), (0, 0.2), (0, 0)]


    for wp in waypoints:
    # while (True)
        try:
            (destination_x, destination_y) = wp
            # destination_x = float(input("Enter x: "))
            # destination_y = float(input("Enter y: "))
            print(f"waypoint destination ({destination_x}, {destination_y})")
            wp = (destination_x * 100, destination_y * 100)

            particles, (oldpos, newpos) = robot.navigate_to_waypoint(wp, particles)
            
            
            
            canvas.drawLine((oldpos[0], oldpos[1], newpos[0], newpos[1]))
            
            particles.draw()

            print("Current Position (calculated by particles): " + str(robot.position))

            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(2)
        
        except KeyboardInterrupt:
            BP.reset_all()