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

# Estimating the overall variance in the straight line motion as about 5 cm
# i.e 160cm -> 5cm
# then for one 10cm straight line motion, the variance is 5/16 = 0.3125
# estimate a very small rotational variance (very little was observed experimentally)
sd_e = 0.25  # s.d. for straight line
sd_f = 0.025  # s.d. for rotation during straight line motion

# estimating overall angular variance as about 20 degrees or about 0.3 radians
# then for one 90 degree turn, the variance is 0.3/4 = 0.075
sd_g = 0.025 # s.d. for rotation


DRAW_SCALE = 15
X_OFFSET = 200
Y_OFFSET = 700


def sample_gaussian(mean=0, std_dev=1):
    return random.gauss(mean, std_dev)

def init_particles(num):
    return [(0, 0, 0, 1/num) for _ in range(num)]

def update_robot_position_straight_line(robot_position, dist):
    (x, y, theta) = robot_position
    x += dist * cos(theta)
    y += dist * sin(theta)
    return (x, y, theta)

def update_robot_position_rotation(robot_position, angle):
    (x, y, theta) = robot_position
    theta += angle
    return (x, y, theta)

def perturb_particle_straight_line(dist, particles):
    for i in range(len(particles)):
        (x, y, theta, w) = particles[i]
        noise_xy = sample_gaussian(std_dev=sd_e)
        noise_theta = sample_gaussian(std_dev=sd_f)
        particles[i] = (  ( x + (dist+noise_xy)*cos(theta)),
                          ( y + (dist+noise_xy)*sin(theta)),
                          theta + noise_theta,
                          w)
    return particles


def perturb_particle_rotation(angle, particles):

    for i in range(len(particles)):
        (x, y, theta, w) = particles[i]
        particles[i] = (x,
                        y,
                        theta + angle + sample_gaussian(std_dev=sd_g),
                        w)

    return particles

def get_robot_position_point_estimate(particles):
    x = 0
    y = 0
    theta = 0
    # weighted mean
    for (x_p, y_p, theta_p, w) in particles:
        x += x_p * w
        y += y_p * w
        theta += theta_p * w
    return (x, y, theta)


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

    print(f"rotated through {degrees} in rotate lolll!")
    target_degrees = degrees # Adjust factor based on calibration

    if degrees < 0:
        speed = -speed


    BP.set_motor_dps(LEFT_MOTOR, -speed)
    BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation

    sleep_time = abs(target_degrees / speed)
    #sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
    sleep_time = sleep_time * ROTATION_TIME_MULTIPLIER
    time.sleep(sleep_time)  # Wait for rotation to complete
    print(f"Slept for f{sleep_time} seconds")
    print(f"Done with rotate at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n  R: {BP.get_motor_encoder(RIGHT_MOTOR)}")




def navigate_to_waypoint(destination, current_position, particles):
    wx, wy = destination
    x, y, theta = current_position
    dx = wx - x
    dy = wy - y
    bearing = atan2(dy, dx)
    angle_to_rotate = (bearing - theta) * (180 / pi) # Convert to degrees for the rotate function
    if angle_to_rotate > 180:
        angle_to_rotate -= 360
    elif angle_to_rotate < -180:
        angle_to_rotate += 360

    distance = sqrt(dx**2 + dy**2)
    print(f"Rotating through angle: {angle_to_rotate}")
    rotate(angle_to_rotate * WHEEL_ROTATION_FOR_1_DEGREE)

    particles = perturb_particle_rotation(bearing-theta, particles)

    BP.set_motor_dps(LEFT_MOTOR, 0)
    BP.set_motor_dps(RIGHT_MOTOR, 0)
    time.sleep(1.5)
    print(f"Driving for distance: {distance}")
    drive_straight_for_distance(distance)

    particles = perturb_particle_straight_line(distance, particles)


    BP.set_motor_dps(LEFT_MOTOR, 0)
    BP.set_motor_dps(RIGHT_MOTOR, 0)
    return (wx, wy, bearing), particles

def draw_square():
    coords = [((0, 0),(0, -40)),
              ((0, -40), (40, -40)),
              ((40, -40), (40, 0)),
              ((40, 0), (0, 0))]

    for (a, b) in coords:
        (x1, y1) = a
        (x2, y2) = b
        line = (x1 * DRAW_SCALE + X_OFFSET,
                y1 * DRAW_SCALE + Y_OFFSET,
                x2 * DRAW_SCALE + X_OFFSET,
                y2 * DRAW_SCALE + Y_OFFSET)
        print("drawLine:" + str(line))

def draw_line(old_position, new_position):
    x1, y1 = old_position[0], -old_position[1]
    x2, y2 = new_position[0], -new_position[1]

    line = (x1 * DRAW_SCALE + X_OFFSET,
            y1 * DRAW_SCALE + Y_OFFSET,
            x2 * DRAW_SCALE + X_OFFSET,
            y2 * DRAW_SCALE + Y_OFFSET)

    print("drawLine:" + str(line))


if __name__ == "__main__":


    # waypoints = [(0.4, 0.4), (0.4, 0.2), (0, 0.2), (0, 0)]
    current_position = (0, 0, 0)
    draw_square()
    particles = init_particles(100)
    print("First 10 particles: " + str(particles[0:10]))
    print("drawParticles:" + str(particles))

    while (True):
        try:
            destination_x = float(input("Enter x: "))
            destination_y = float(input("Enter y: "))
            wp = (destination_x * 100, destination_y * 100)

            new_position, particles = navigate_to_waypoint(wp, current_position, particles)

            draw_line(current_position, new_position)


            print("First 10 particles: " + str(particles[0:10]))
            draw_particles = [(x * DRAW_SCALE + X_OFFSET, y * DRAW_SCALE + Y_OFFSET, theta, w) 
                              for (x, y, theta, w) in particles]
            print("Drawing the particles now !!!!!!")
            print("drawParticles:" + str(draw_particles))

            current_position = get_robot_position_point_estimate(particles)
            print("Current Position: " + str(current_position))

            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(2)
        
        except KeyboardInterrupt:
            BP.reset_all()