# 52x46cm square
import math


def get_walls():
    return []


def get_facing_wall(x, y, theta):
    """
    Finds the wall that the sonar beam will hit first and returns its coordinates along with the distance.

    :param x: float - x-coordinate of the robot
    :param y: float - y-coordinate of the robot
    :param theta: float - orientation of the robot in radians
    :asm: get_walls() is a function that returns a list of walls (each wall as (x1, y1, x2, y2))
    :return: tuple (x1, y1, x2, y2, m) where (x1, y1, x2, y2) are the coordinates of the closest intersecting wall
             and m is the distance from (x, y) to the intersection point. Returns None if no intersection.
    """
    walls = get_walls()
    min_distance = float('inf')
    closest_wall = None

    # Define sonar beam as a half-line starting at (x, y) extending in direction theta
    dx = math.cos(theta)
    dy = math.sin(theta)

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
            distance = math.sqrt((ix - x) ** 2 + (iy - y) ** 2)

            if distance < min_distance:
                min_distance = distance
                closest_wall = (x1, y1, x2, y2)

    if closest_wall:
        return *closest_wall, min_distance
    return None


def calculate_likelihood(x, y, theta, z):
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

    #


    # look at the difference between m and the actual measurement z and calculate a likelihood value using a
    # Gaussian model (with a constant added to make it more robust).

    # Use standard deviation set according to what you learned about the sonar in last week’s calibration exercise
    # — I would probably use around 2–3cm to be a bit conservative.
    '''
    The function could be made more sophisticated by also checking whether the incidence angle is going
    to be too big to get a sensible sonar reading. If too many particles say that this is the case, probably the
    best thing is to skip the update step entirely on this step. This is probably not needed this week because
    the trajectory for the robot to follow is designed so that it will usually be looking at walls face-on.
    '''
    pass

