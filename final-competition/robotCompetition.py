import brickpi3
import time
import cv2 
import numpy as np
from picamera2 import Picamera2
import math
 
BP = brickpi3.BrickPi3()
 

picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
 
picam2.start()
 
starttime = time.time()
 
white = (255,255,255)
# Constants and variables
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.015
ROBOTRADIUS = 0.09
W = 2 * ROBOTRADIUS # width of robot
SAFEDIST = 0.1      # used in the cost function for avoiding obstacles
LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_C

MAXVELOCITY = 5     #ms^(-1) max speed of each wheel
MAXACCELERATION = 0.5 #ms^(-2) max rate we can change speed of each wheel

# Timestep delta to run control and simulation at
TAU = 0.1

obstacles = []
# Starting wheel velocities
vL = 0.5
vR = 0.5


# Starting pose of robot
x = 0.0
y = 0.0
theta = 0.0

target = (3.0, 0)  # Target location for robot to reach


# Homography for camera: CHANGE THESE NUMBERS: enter your own correspondences 
# to calibrate the ground plane homography for your robot
(x1, y1, u1, v1) = (48, 12, 145, 247)
(x2, y2, u2, v2) = (48, -14, 501, 251)
(x3, y3, u3, v3) = (26, 8, 99, 453)
(x4, y4, u4, v4) = (26, -8, 523, 452)

# Form and solve linear system 
A = np.array([
    [x1, y1, 1, 0, 0, 0, -u1 * x1, -u1 * y1],
    [0, 0, 0, x1, y1, 1, -v1 * x1, -v1 * y1],
    [x2, y2, 1, 0, 0, 0, -u2 * x2, -u2 * y2],
    [0, 0, 0, x2, y2, 1, -v2 * x2, -v2 * y2],
    [x3, y3, 1, 0, 0, 0, -u3 * x3, -u3 * y3],
    [0, 0, 0, x3, y3, 1, -v3 * x3, -v3 * y3],
    [x4, y4, 1, 0, 0, 0, -u4 * x4, -u4 * y4],
    [0, 0, 0, x4, y4, 1, -v4 * x4, -v4 * y4]
])

b = np.array([u1, v1, u2, v2, u3, v3, u4, v4])
R, residuals, RANK, sing = np.linalg.lstsq(A, b, rcond=None)

# Build homography matrix
H = np.array([
    [R[0], R[1], R[2]],
    [R[3], R[4], R[5]],
    [R[6], R[7], 1]
])

print ("Homography")
print (H)

    # Inverse homography
HInv = np.linalg.inv(H)


def predictPosition(vL, vR, x, y, theta, deltat):
        # Simple special cases
        # Straight line motion
        if (vL == vR): 
                xnew = x + vL * deltat * math.cos(theta)
                ynew = y + vL * deltat * math.sin(theta)
                thetanew = theta
        # Pure rotation motion
        elif (vL == -vR):
                xnew = x
                ynew = y
                thetanew = theta + ((vR - vL) * deltat / W)
        else:
                # Rotation and arc angle of general circular motion
                # Using equations given in Lecture 2
                R = W / 2.0 * (vR + vL) / (vR - vL)
                deltatheta = (vR - vL) * deltat / W
                xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
                ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
                thetanew = theta + deltatheta

        return (xnew, ynew, thetanew)
                
                
# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
        closestdist = 100000.0  
        # Calculate distance to closest obstacle
        for obstacle in obstacles:
                # Is this a barrier we know about? barrier[2] flag is set when sonar observes it
                if(obstacle[2] == 1):
                        dx = obstacle[0] - x
                        dy = obstacle[1] - y
                        d = math.sqrt(dx**2 + dy**2)
                        # Distance between closest touching point of circular robot and circular barrier
                        dist = d - BARRIERRADIUS - ROBOTRADIUS
                        if (dist < closestdist):
                                closestdist = dist
        return closestdist


# Functions to transform via the forward and inverse homography
def HtransformXYtoUV(H, xin, yin):
    xvec = np.array([xin, yin, 1])
    uvec = H.dot(xvec)
    uout = uvec[0]/uvec[2]
    vout = uvec[1]/uvec[2]
    return(uout, vout)


def HtransformUVtoXY(HInv, uin, vin):
    uvec = np.array([uin, vin, 1])
    xvec = HInv.dot(uvec)
    xout = xvec[0]/xvec[2]
    yout = xvec[1]/xvec[2]
    return(xout, yout)


def observeObstacles(x, y, theta):
    for i in range(10):
        img = picam2.capture_array()

        # Convert to HSV colour space    
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        # Apply colour thresholding: for red this is done in two steps
        # lower mask (0-10)
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red) 
        # upper mask (170-180)
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # join my masks
        mask = mask0+mask1
        # This is a thresholded version of the image which you can display if
        # you want to check what the colour thresholding does
        result = cv2.bitwise_and(img, img, mask=mask)

        # Calculate connected components: colour thresholded "blob" regions 
        output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32F)
        (numLabels, labels, stats, centroids) = output

        obstacles = []
        # Find the properties of the detected blobs
        for i in range(0, numLabels):
            # i=0 is the background region so ignore it
            if i != 0:
                
                # Extract the connected component statistics and centroid
                # Here you can get the limits of the blob if you need them
                x = stats[i, cv2.CC_STAT_LEFT]
                y = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]
                area = stats[i, cv2.CC_STAT_AREA]
                (cu, cv) = centroids[i]
                
                
                
                # calculate world position (i.e. relative to origin)
                (cx, cy) = HtransformUVtoXY(HInv, cu, cv)
                ncx = x + math.cos(theta) * cx - math.sin(theta) * cy
                ncy = y + math.sin(theta) * cx + math.cos(theta) * cy
                (cx, cy) = (ncx, ncy)
                
                
                
                # Print out the properties of blobs above a certain size
                # Only consider obstacles with a significant size
                if area > 150:
                    # Check for overlap with existing obstacles
                    is_overlapping = False
                    for existing_obstacle in obstacles:
                        # Calculate distance between current detected obstacle and each existing obstacle
                        dist = np.sqrt((cx - existing_obstacle[0])**2 + (cy - existing_obstacle[1])**2)

                        if dist < (ROBOTRADIUS + 0.05):  # Adjust the threshold as necessary
                            is_overlapping = True
                            break

                    # If not overlapping, add to the obstacle list
                    if not is_overlapping:
                        obstacles.append((cx, cy))  # You can store centroids or other info here


        print(f"Obstacles: {obstacles}")
        return obstacles


while(1):
        BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
        BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))
        
        # Check if any new barriers are visible from current pose
        observeObstacles(x, y, theta)

        # Planning
        # We want to find the best benefit where we have a positive component for closeness to target,
        # and a negative component for closeness to obstacles, for each of a choice of possible actions
        bestBenefit = -100000
        FORWARDWEIGHT = 12
        OBSTACLEWEIGHT = 16

        # Range of possible motions: each of vL and vR could go up or down a bit
        vLpossiblearray = (vL - MAXACCELERATION * TAU, vL, vL + MAXACCELERATION * TAU)
        vRpossiblearray = (vR - MAXACCELERATION * TAU, vR, vR + MAXACCELERATION * TAU)

        for vLpossible in vLpossiblearray:
                for vRpossible in vRpossiblearray:
                        # We can only choose an action if it's within velocity limits
                        if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                                # Predict new position in TAU seconds
                                TAU = 1.5 
                                (xpredict, ypredict, thetapredict) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)

                                # What is the distance to the closest obstacle from this possible position?
                                distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
                                # Calculate how much close we've moved to target location
                                previousTargetDistance = math.sqrt((x - target[0])**2 + (y - target[1])**2)
                                newTargetDistance = math.sqrt((xpredict - target[0])**2 + (ypredict - target[1])**2)
                                distanceForward = previousTargetDistance - newTargetDistance
                                # Alternative: how far have I moved forwards?
                                # distanceForward = xpredict - x
                                # Positive benefit
                                distanceBenefit = FORWARDWEIGHT * distanceForward
                                # Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
                                if (distanceToObstacle < SAFEDIST):
                                        obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                                else:
                                        obstacleCost = 0.0
                                # Total benefit function to optimise
                                benefit = distanceBenefit - obstacleCost
                                if (benefit > bestBenefit):
                                        vLchosen = vLpossible
                                        vRchosen = vRpossible
                                        bestBenefit = benefit

        vL = vLchosen
        vR = vRchosen
        print(f"new vL: {vL}, new vR: {vR}")
        BP.set_motor_dps(LEFT_MOTOR, vL)
        BP.set_motor_dps(RIGHT_MOTOR, vR)
        time.sleep(TAU)
        (xpredict, ypredict, thetapredict) = predictPosition(vL, vR, x, y, theta, TAU)
        print(f"new x: {xpredict}, new y: {ypredict}, new theta: {thetapredict}")
        x = xpredict
        y = ypredict
        theta = thetapredict
