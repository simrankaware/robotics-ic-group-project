#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
import bisect
from math import pi, atan2, sqrt, cos, sin, sqrt, e 
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import random

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
# Configure for an NXT ultrasonic sensor.
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C
WHEEL_RADIUS = 3.05
AXLE_RADIUS = 8.5
WHEEL_ROTATION_FOR_1_DEGREE = ((AXLE_RADIUS * pi) / 2) / (2 * pi * WHEEL_RADIUS) * 4

# Parameters
BASE_POWER = -15
kp = 1.5

TIME_DELAY = 300 / (120 * pi)
LINE_TIME_MULTIPLIER = 0.89
ROTATION_TIME_MULTIPLIER = 0.88
ANG_VELOCITY = 350
ANGLE_CALIBRATION = 39.75



DRAW_SCALE = 10
X_OFFSET = 200
Y_OFFSET = 500

# Constants for the likelihood function
K = 0.01   # Constant added to Gaussian likelihood function to make it more robust
SIGMA = 2  # Standard deviation of the Gaussian likelihood function
SONAR_PORT = BP.PORT_1  # Port for the sonar sensor


def get_facing_wall(x, y, theta, map):
    """
    Finds the wall that the sonar beam will hit first and returns its coordinates along with the distance.
    """
    walls = map.walls
    min_distance = float('inf')
    closest_wall = None

    # Sonar beam as a half-line from (x, y) in direction theta.
    dx = cos(theta)
    dy = sin(theta)

    def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None  # Parallel lines

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

        if 0 <= t <= 1 and u >= 0:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return ix, iy
        return None

    for wall in walls:
        x1, y1, x2, y2 = wall
        intersection = line_intersection(x1, y1, x2, y2, x, y, x + dx, y + dy)
        if intersection:
            ix, iy = intersection
            distance = sqrt((ix - x) ** 2 + (iy - y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_wall = (x1, y1, x2, y2)

    if closest_wall:
        return closest_wall, min_distance
    return None

            
def sample_gaussian(mean=0, std_dev=1):
        return random.gauss(mean, std_dev)


def gaussian_likelihood(z, m, sigma, K=0):
    """
    Gaussian function with mean m and standard deviation sigma.
    (Note: The absolute scale is not critical because we normalize later.)
    """
    return e ** ((-(z - m)**2) / (2 * sigma**2)) + K


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
        
        
        self.data = [(84, 30, 0, 1/self.n) for _ in range(self.n)]
        
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

    def calculate_likelihood(self, x, y, theta, z, map):
        """
        For a given particle at (x,y,theta) and a sonar measurement z, determine the expected
        measurement m from the wall that the sonar beam would hit, and return a likelihood value.
        """
        output = get_facing_wall(x, y, theta, map)
        if output is None:
            # If no wall is hit, return a very small likelihood
            return 0
        
        closest_wall, m = output
        likelihood = gaussian_likelihood(z, m, SIGMA, K)
        return likelihood   

    def measurement_update(self, z, map):
        """
        Update the particle weights based on the likelihood of the sonar measurement z.
        """

        zeroes = 0
        weights = []
        for i in range(len(self.data)):
            (x, y, theta, w) = self.data[i]
            w = self.calculate_likelihood(x, y, theta, z, map)
            if w == 0:
                zeroes += 1
            weights.append(w)
             
        if zeroes >= 50:
            print("Too many zeroes, not updating weights")
            pass
        else:   
            for i in range(len(weights)):
                self.data[i] = (self.data[i][0], self.data[i][1], self.data[i][2], weights[i])
        
                # Normalize weights
                total = sum([w for (_, _, _, w) in self.data])
                self.data = [(x, y, theta, w/total) for (x, y, theta, w) in self.data]
        
    
    def resample_particles(self):
        """
        Resample particles based on their weights using a cumulative weight array and binary search.
        After resampling, reset each particle's weight to 1/n.
        """
        cumulative_weights = []
        cumulative_sum = 0
        for particle in self.data:
            cumulative_sum += particle[3]
            cumulative_weights.append(cumulative_sum)
        
        new_particles = []
        for _ in range(self.n):
            r = random.random()
            # Find the index where r would be inserted to keep the cumulative_weights sorted.
            idx = bisect.bisect_left(cumulative_weights, r)
            new_particles.append(self.data[idx])
        
        # Reset weights to uniform 1/n.
        self.data = [(x, y, theta, 1/self.n) for (x, y, theta, _) in new_particles]

    def draw(self):
        canvas.drawParticles(self.data)

        
        
class Robot:
    def __init__(self):
        # self.position = (0, 0, 0)
        self.position=(84, 30, 0)
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
        print(f"Rotations: {rotations} | Distance: {distance}")
        target_degrees = rotations * 360
        BP.set_motor_dps(LEFT_MOTOR, speed)   # Set slow speed
        BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation
        sleep_time = abs(target_degrees / speed)
        sleep_time = sleep_time * LINE_TIME_MULTIPLIER
        print(f"Sleep time: {sleep_time}")
        time.sleep(sleep_time)  # Wait for rotation to complete


    def get_sonar_reading(self):
        time.sleep(1.5)
        sonar_value = None
        while sonar_value is None:
            try:
                sonar_value = BP.get_sensor(SONAR_PORT)
                print(f"Sensor value: {sonar_value}")
            except brickpi3.SensorError:
                pass

    def rotate(self, degrees, speed=ANG_VELOCITY):  # Add a speed parameter (default: 50 dps)
        BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
        BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))
        target_degrees = degrees # Adjust factor based on calibration
        if degrees < 0:
            speed = -speed
        BP.set_motor_dps(LEFT_MOTOR, -speed)
        BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation
        sleep_time = abs(target_degrees / speed)
        sleep_time = sleep_time * ROTATION_TIME_MULTIPLIER
        time.sleep(sleep_time)  # Wait for rotation to complete
            

    def get_sensor_reading(self):   
        sonar_value = None
        while sonar_value is None:
            time.sleep(0.5)
            try:
                sonar_value = BP.get_sensor(SONAR_PORT)
                print(f"Sensor value: {sonar_value}")
            except brickpi3.SensorError:
                pass
        return sonar_value
    

    def mcl_navigate_to_waypoint(self, destination, particles, map):
        wx, wy = destination
        x, y, theta = self.position
        print(f"Starting MCL at ({x}, {y}, {theta}), navigating to ({wx}, {wy})")


    def angle_and_distance(self, destination):

        wx, wy = destination 
        x, y, theta = self.position

        print(f"Robot at ({x}, {y}, {theta}), navigating to ({wx}, {wy})")
        dx = wx - x
        dy = wy - y
        bearing = atan2(dy, dx)
        angle_to_rotate = (bearing - theta) * (180 / pi)
        distance = sqrt(dx**2 + dy**2)
        if angle_to_rotate > 180:
            angle_to_rotate -= 360
        elif angle_to_rotate < -180:
            angle_to_rotate += 360
        return angle_to_rotate, distance


       
    def navigate_to_waypoint(self, destination, particles, map, canvas):


        margin = 3  # Define a margin of error in cm
        while abs(self.position[0] - destination[0]) > margin or abs(self.position[1] - destination[1]) > margin:
            angle_to_rotate, distance = self.angle_and_distance(destination)
        
            print(f"Angle to rotate: {angle_to_rotate} | Distance to drive: {distance}")

            # rotation logic
            self.rotate(angle_to_rotate * WHEEL_ROTATION_FOR_1_DEGREE)
            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(1.5)

            # Pass the angle in radians to perturb_particle_rotation
            particles.perturb_particle_rotation(angle_to_rotate * pi/180)


            # drive straight logic
            distance = min(distance, 20)
            print("Driving for distance: " + str(distance))
            self.drive_straight_for_distance(distance)
            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(1.5)

            # particle update logic
            particles.perturb_particle_straight_line(distance)
            sonar_value = self.get_sensor_reading()
            particles.measurement_update(sonar_value + 15, map)
            particles.draw()


            # update position
            estim_position = particles.get_point_estimate()
            print(f"Robot believes it is here: {estim_position}")
            canvas.drawLine((self.position[0], self.position[1], estim_position[0], estim_position[1])) 
            old_position = self.position
            self.position = estim_position
            print(f"robot has moved from {old_position} to {estim_position}")

        



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
    print(f"Initial Position Estimate: {particles.get_point_estimate()}")
    print(f"Initial Position: {robot.position}")

# 1. (84, 30)
# 2. (180, 30)
# 3. (180, 54)
# 4. (138, 54)
# 5. (138, 168)
# 6. (114, 168)
# 7. (114, 84)
# 8. (84, 84)
# 9. (84, 30)
    waypoints = [(1.8, 0.3), (1.8, 0.54), (1.38, 0.54), (1.38, 1.68), (1.14, 1.68), (1.14, 0.84), (0.84, 0.84), (0.84, 0.3)]


    for wp in waypoints:
    # while (True)
        try:
            print("\nWAYPOINT ----------------")

            (destination_x, destination_y) = wp
            # destination_x = float(input("Enter x: "))
            # destination_y = float(input("Enter y: "))
            print(f"waypoint destination ({destination_x}, {destination_y})")
            wp = (destination_x * 100, destination_y * 100)

            robot.navigate_to_waypoint(wp, particles, mymap, canvas)
            # sonar_value = robot.get_sensor_reading
            
            # print(f"Sensor value: {sonar_value + 15}")
            # particles.measurement_update(sonar_value + 15, mymap)
            
            print("Current Position (calculated by particles): " + str(robot.position))

            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(2)
        
        except KeyboardInterrupt:
            BP.reset_all()
        

