from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
from math import pi, cos, sin
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import random

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C
WHEEL_RADIUS = 3.05
AXLE_RADIUS = 6.5
WHEEL_ROTATION_FOR_90_DEGREES = ((AXLE_RADIUS * pi) / 2) / (2 * pi * WHEEL_RADIUS) * 360

# Parameters
BASE_POWER = -15
kp = 1.5

TIME_DELAY = 300 / (120 * pi) 

ANGLE_CALIBRATION = 39.75

DISTANCE = 10 # in cm
ANG_VELOCITY = -120
SLEEP = 10

sd_e = 0.25  # s.d. for straight line
sd_f = 0.025  # s.d. for rotation during straight line motion
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


def drive_straight_for_time(duration):
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

def drive_straight_for_distance(distance, speed=-50): # distance in cm
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


def rotate(degrees, speed=50):  # Add a speed parameter (default: 50 dps)
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

    target_degrees = degrees # Adjust factor based on calibration

    BP.set_motor_dps(LEFT_MOTOR, -speed)   # Set slow speed
    BP.set_motor_dps(RIGHT_MOTOR, speed) # Opposite direction for rotation

    sleep_time = abs(target_degrees / speed)
    sleep_time = sleep_time - TIME_DELAY if TIME_DELAY < sleep_time else 0
    time.sleep(sleep_time)  # Wait for rotation to complete
    print(f"Slept for f{sleep_time} seconds")
    print(f"Done with rotate at\n L: {BP.get_motor_encoder(LEFT_MOTOR)}\n  R: {BP.get_motor_encoder(RIGHT_MOTOR)}")




if __name__ == "__main__":
    try:
        particles = init_particles(100)
        robot_position = (0, 0, 0)
        print("drawParticles:" + str(particles))
        for _ in range(4):
            for _ in range(4):
                drive_straight_for_distance(DISTANCE, ANG_VELOCITY)
                BP.set_motor_dps(LEFT_MOTOR, 0)
                BP.set_motor_dps(RIGHT_MOTOR, 0)
                particles = perturb_particle_straight_line(DISTANCE, particles)
                draw_particles = [(x * DRAW_SCALE+X_OFFSET, y*DRAW_SCALE+Y_OFFSET, theta, w) for (x, y, theta, w) in particles]

                new_robot_position = update_robot_position_straight_line(robot_position, DISTANCE)
                print("First 10 particles: " + str(particles[0:10]))
                print("drawParticles:" + str(draw_particles))
                line = (robot_position[0] * DRAW_SCALE + X_OFFSET, 
                        robot_position[1] * DRAW_SCALE + Y_OFFSET, 
                        new_robot_position[0] * DRAW_SCALE + X_OFFSET, 
                        new_robot_position[1] * DRAW_SCALE + Y_OFFSET)
                print("line:" + str(line))
                print("drawLine:" + str(line))
                robot_position = new_robot_position
                print("Robot position: " + str(robot_position))
                time.sleep(2)


            rotate(WHEEL_ROTATION_FOR_90_DEGREES)
            BP.set_motor_dps(LEFT_MOTOR, 0)
            BP.set_motor_dps(RIGHT_MOTOR, 0)
            time.sleep(2)
            particles = perturb_particle_rotation(-pi/2, particles)
            draw_particles = [(x*DRAW_SCALE+X_OFFSET, y*DRAW_SCALE+Y_OFFSET, theta, w) for (x, y, theta, w) in particles]

            new_robot_position = update_robot_position_rotation(robot_position, -pi/2)
            print("First 10 particles: " + str(particles[0:10]))
            print("drawParticles:" + str(draw_particles))
            robot_position = new_robot_position
            print("Robot position: " + str(robot_position))
        BP.reset_all()
    except KeyboardInterrupt as e:
        BP.reset_all()