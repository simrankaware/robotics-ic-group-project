#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from math import pi, atan2, sqrt, cos, sin, sqrt, e
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import random

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# Configure for an NXT ultrasonic sensor.
# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
# BP.PORT_1 specifies that the sensor will be on sensor port 1.
# BP.SENSOR_TYPE.NXT_ULTRASONIC specifies that the sensor will be an NXT ultrasonic sensor.
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
LINE_TIME_MULTIPLIER = 0.9
ROTATION_TIME_MULTIPLIER = 0.88
ANG_VELOCITY = 350
ANGLE_CALIBRATION = 39.75


# Constants
K = 0  # Constant added to Gaussian likelihood function to make it more robust
SIGMA = 2  # Standard deviation of the Gaussian likelihood function

            
def sample_gaussian(mean=0, std_dev=1):
        return random.gauss(mean, std_dev)

def gaussian_likelihood(z, m, sigma, K=0):
    """
    Gaussian function with mean mu and standard deviation sigma.
    """
    return e ** ((-(z - m)**2) / (2 * sigma**2)) + K
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

    





    def calculate_likelihood(self, x, y, theta, z, map):
        """
        update function which every time the sonar makes a measurement loops
        through all of your particles and updates each one’s weight by multiplying
        it by an appropriate likelihood function which corresponds to how well
        that particle agrees with the sonar reading
        :param x: x coordinate of particle estimate
        :param y: y coordinate of particle estimate
        :param theta: bearing of particle estimate (degrees)
        :param z: sonar measurement
        """
        # find out which wall the sonar beam would hit if the robot is at position (x, y, θ),
        # and then the expected depth measurement m that should be recorded.
        # get m from get_facing_wall()

        closest_wall, m = get_facing_wall(x, y, theta, map)


        # look at the difference between m and the actual measurement z and calculate a likelihood value using a
        # Gaussian model (with a constant added to make it more robust).

        # The likelihood function should be a Gaussian with mean m and standard deviation σ.
        # The likelihood value should be the value of the Gaussian at z.
        # The Gaussian function is given by:
        # f(x) = 1/(σ√2π) * e^(-(x-m)^2/(2σ^2))
        # where x is the actual measurement z, m is the expected measurement m, and σ is the standard deviation.
        # The standard deviation σ should be set according to what you learned about the sonar in last week’s calibration exercise.
        # You can use a standard deviation of around 2–3cm to be a bit conservative.

        likelihood = gaussian_likelihood(z, m, SIGMA, K)

        # Use standard deviation set according to what you learned about the sonar in last week’s calibration exercise
        # — I would probably use around 2–3cm to be a bit conservative.

        '''
        The function could be made more sophisticated by also checking whether the incidence angle is going
        to be too big to get a sensible sonar reading. If too many particles say that this is the case, probably the
        best thing is to skip the update step entirely on this step. This is probably not needed this week because
        the trajectory for the robot to follow is designed so that it will usually be looking at walls face-on.
        '''

        # the actual particles' weights need to be updated if the likelihood is not zero
        
        return likelihood
    
    def resample_particles(self):
        """
        Resample particles based on their weights using the systematic resampling method.
        This method ensures that the more likely particles (those with higher weights) are selected more frequently.
        """
        # Step 1: Calculate the cumulative weight array
        cumulative_weights = []
        
        cumulative_sum = 0
        for particle in self.data:
            cumulative_sum += particle[3]
            cumulative_weights.append(cumulative_sum)
        
        # Step 2: Resample particles
        new_particles = []
        i = 0  # Index to keep track of particles
        
        # Generate N random numbers in the range [0, 1]
        random_vals = [random.random() for _ in range(self.n)]
        
        for val in random_vals:
            # Step 3: Find the corresponding particle using the cumulative weights
            while i < self.n and val > cumulative_weights[i]:
                i += 1
            # Add the particle corresponding to the random value
            new_particles.append(self.data[i])
        
        # Step 4: Replace the old particle set with the new one
        ### CHECK THIS (DO WE RESET THE WEIGHTS?)
        self.data = new_particles

    def measurement_update(self, z, map):
        """
        Update the particles' weights based on the sonar measurement.
        """
        new_particles = []
        for particle in self.data:
            x, y, theta, w = particle
            likelihood = self.calculate_likelihood(x, y, theta, z, map)
            print("Likelihood: " + str(likelihood))
            w *= likelihood
            print(w)
            new_particles.append((x, y, theta, w))

        self.data = []
        # Normalize the weights
        total_weight = sum(w for (_, _, _, w) in new_particles)
        print("Total Weight:" + str(total_weight))

        for (x, y, z, w) in new_particles:
            self.data.append((x, y, z, w / total_weight))

        # Resample
        self.resample_particles()
    
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
        self.update_robot_position_straight_line(distance)


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
        self.update_robot_position_rotation(degrees * (pi / 180))
            

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


def get_facing_wall(x, y, theta, map):
    """
    Finds the wall that the sonar beam will hit first and returns its coordinates along with the distance.

    :param x: float - x-coordinate of the robot
    :param y: float - y-coordinate of the robot
    :param theta: float - orientation of the robot in radians
    :asm: get_walls() is a function that returns a list of walls (each wall as (x1, y1, x2, y2))
    :return: tuple (x1, y1, x2, y2, m) where (x1, y1, x2, y2) are the coordinates of the closest intersecting wall
             and m is the distance from (x, y) to the intersection point. Returns None if no intersection.
    """
    walls = map.walls
    min_distance = float('inf')
    closest_wall = None

    # Define sonar beam as a half-line starting at (x, y) extending in direction theta
    dx = cos(theta)
    dy = sin(theta)

    def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
        """Returns the intersection point of two lines if they intersect within segment bounds."""
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None  # Parallel lines

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

        if 0 <= t <= 1 and u >= 0:
            # Intersection point
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

            sensor_value = None
            while sensor_value is None:
                try:
                    sensor_value = BP.get_sensor(BP.PORT_1)
                except brickpi3.SensorError as error:
                    print(f"Sensor error: {error}. Retrying...")

            print("Sensor value: " + str(sensor_value))
            particles.measurement_update(sensor_value, mymap)
                      
            canvas.drawLine((oldpos[0], oldpos[1], newpos[0], newpos[1]))
            
            particles.draw()

            print("Current Position (calculated by particles): " + str(robot.position))

            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(2)
        
        except KeyboardInterrupt:
            BP.reset_all()