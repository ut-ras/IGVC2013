import math

# used as a shortcut/optimizer
pi2 = math.pi*2.0

# these constants are in meters
MAX_VAL = 1.0 # ranges larger in distance than this are set to this
CLOSE_ENOUGH_TO_GOAL = .1
MAX_VAL_PRECISION = 1e-3 # precision around max
MIN_CLEARANCE_ALLOWED = .8 # set to approximate width of robot
MIN_EDGE_CLEARANCE = 1.0 # clearance that must be met before added additional directions to a gap
MIN_VAL = 2e-2 # if ranges are below this, assume they are actually max values
MAXIMUM_CLEARANCE = MAX_VAL*2.0 # used to normalize clearances
ANGLE_PRECISION = 1e-1

MAX_V = .5
MAX_ANGULAR = .2
"""
CLEARANCE_WEIGHT = .3
CURRENT_HEADING_WEIGHT = .1
GOAL_HEADING_WEIGHT = .6
"""
CLEARANCE_WEIGHT = .19
CURRENT_HEADING_WEIGHT = .01
GOAL_HEADING_WEIGHT = .8

# returns angle equivalent to argument, but in range [0, 2*pi]
def boundAngleTo2PI(angle):
    return (angle%pi2 + pi2)%pi2;

# returns angle equivalent to argument, but in range [-pi, pi]
def boundAngleToNPItoPI(angle):
    angle = boundAngleTo2PI(angle);
    if angle > math.pi:
        angle -= pi2
    return angle

# expects p0 and p1 to be objects with x & y fields
def euclidDistPoint(p0, p1):
    return math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

# expects p0 and p1 to be [x, y]
def euclidDistArr(p0, p1):
    return math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)

# returns the angle difference (a1 - a2) in the range [-pi, pi]
def angleDif(a1, a2):
    a1 = boundAngleTo2PI(a1)
    a2 = boundAngleTo2PI(a2)
    r = boundAngleTo2PI(a1 - a2)

    if r > math.pi:
        r -= pi2

    return r

# estimates the distance between two edges of a gap
def calcClearance(angle1, dist1, angle2, dist2) :
    dist = min(dist1, dist2)

    p0 = (dist*math.cos(angle1), dist*math.sin(angle1))
    p1 = (dist*math.cos(angle2), dist*math.sin(angle2))

    return euclidDistArr(p0, p1)

# returns angle on range [-pi to pi]
def averageAngles(angle1, angle2):
    xcomp = (math.cos(angle1) + math.cos(angle2))/2.0
    ycomp = (math.sin(angle1) + math.sin(angle2))/2.0

    return math.atan2(ycomp, xcomp)

def rotateDirections(viableDirections, heading):
    for i in range(len(viableDirections)):
        viableDirections[i].direction = \
            boundAngleTo2PI(viableDirections[i].direction + heading)
