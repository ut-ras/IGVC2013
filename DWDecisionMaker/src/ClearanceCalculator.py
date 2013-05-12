import math
#from Geometry import GLib

LARGE_DISTANCE = 1e6
SMALL_ENOUGH = 1e-6

class Circle:
    def __init__(self, x, y, r):
        self.type = "circle"
        self.x = float(x)
        self.y = float(y)
        self.r = float(r)

class Point:
    def __init__(self, x, y):
        self.type = "point"
        self.x = float(x)
        self.y = float(y)

class Vector:
    def __init__(self, x, y, dir):
        self.type = "vector"
        self.x = float(x)
        self.y = float(y)
        self.dir = float(dir)

class Pose:
    def __init__(self, x, y, dir, v, w):
        self.x = float(x)
        self.y = float(y)
        self.dir = float(dir)
        self.v = float(v)
        self.w = float(w)

class TrajIntersection:
    def __init__(self, point, delta):
        self.point = point
        self.delta = float(delta)
        self.traj = None
class CLib:
    @staticmethod
    def makeCircle(x, y, r):
        return Circle(x, y, r)

    @staticmethod
    def makePoint(x, y):
        return Point(x, y)

    @staticmethod
    def makeVector(x, y, dir):
        return Vector(x, y, dir)

    @staticmethod
    def makePose(x, y, dir, v, w):
        return Pose(x, y, dir, v, w)

    @staticmethod
    def getParameterOfIntersection(x0, y0, theta0, x1, y1, theta1):
        # return False if the angles are parallel
        if abs(math.sin(theta0 - theta1)) < SMALL_ENOUGH:
            return False

        return -(math.sin(theta0)*(x1 - x0) + math.cos(theta0)*(y0 - y1)) / math.sin(theta0 - theta1)

    @staticmethod
    def circleIntersections(c1, c2):
        dist = GLib.euclid(c1.x, c1.y, c2.x, c2.y)
        angle = math.atan2(c2.y - c1.y, c2.x - c1.x)

        small = c1.r if (c1.r < c2.r) else c2.r
        big = c1.r if (c1.r > c2.r) else c2.r

        if c1.r == c2.r and c1.x == c2.x and c1.y == c2.y:
            return []
        elif dist == small + big or dist == big - small:
            return [Point(c1.x + c1.r*math.cos(angle),
                          c1.y + c1.r*math.sin(angle))]
        elif dist < small + big and dist > big - small:
            # using the law of cosines
            angleOffset = math.acos((c1.r*c1.r + dist*dist - c2.r*c2.r)/(2.0*c1.r*dist))

            return [
                Point(c1.x + c1.r*math.cos(angle + angleOffset),
                      c1.y + c1.r*math.sin(angle + angleOffset)),
                Point(c1.x + c1.r*math.cos(angle - angleOffset),
                      c1.y + c1.r*math.sin(angle - angleOffset))
            ]

        return []

    # returns circle, vector, or point
    @staticmethod
    def calculateTrajetory(pose):
        if abs(pose.v) < SMALL_ENOUGH:
            return Point(pose.x, pose.y)
        elif abs(pose.w) <= SMALL_ENOUGH:
            return Vector(pose.x, pose.y, pose.dir)

        # otherwise, v and w are both nonzero:
        radius = abs(pose.v/pose.w)

        if pose.v > 0:
            if pose.w > 0:
                return Circle(
                    pose.x + radius*math.cos(pose.dir + math.pi/2.0),
                    pose.y + radius*math.sin(pose.dir + math.pi/2.0),
                    radius
                    )
            else:
                return Circle(
                    pose.x + radius*math.cos(pose.dir - math.pi/2.0),
                    pose.y + radius*math.sin(pose.dir - math.pi/2.0),
                    radius
                    )
        else:
            raise Exception("exception: not implemented yet!")

    @staticmethod
    def trajectoryIntersection(pose, traj, circle):
        if traj.type == "circle":
            points = CLib.circleIntersections(traj, circle)

            if len(points) == 0:
                return False
            elif len(points) == 1:
                return points[0]
            elif len(points) == 2:
                # need to find the closest of the two points to the starting point
                # along the trajectory
                curAngle = GLib.boundAngle0to2Pi(math.atan2(
                    pose.y - traj.y,
                    pose.x - traj.x
                    ))
                angle1 = GLib.boundAngle0to2Pi(math.atan2(
                    points[0].y - traj.y,
                    points[0].x - traj.x
                    ))
                angle2 = GLib.boundAngle0to2Pi(math.atan2(
                    points[1].y - traj.y,
                    points[1].x - traj.x
                    ))

                if pose.w > 0:
                    angle1 = angle1 + math.pi*2 if (angle1 < curAngle) else angle1
                    angle2 = angle2 + math.pi*2 if (angle2 < curAngle) else angle2

                    if angle1 < angle2:
                        return TrajIntersection(
                            points[0],
                            traj.r*(angle1 - curAngle)
                            )
                    else:
                        return TrajIntersection(
                            points[1],
                            traj.r*(angle2 - curAngle)
                            )
                else:
                    angle1 = angle1 - math.pi*2 if (angle1 > curAngle) else angle1
                    angle2 = angle2 - math.pi*2 if (angle2 > curAngle) else angle2

                    if angle1 > angle2:
                        return TrajIntersection(
                            points[0],
                            traj.r*(curAngle - angle1)
                            )
                    else:
                        return TrajIntersection(
                            points[1],
                            traj.r*(curAngle - angle2)
                            )
        elif traj.type == "vector":
            vector = Vector(circle.x, circle.y, traj.dir + math.pi/2.0)

            s = CLib.getParameterOfIntersection(
                vector.x, vector.y, vector.dir,
                traj.x, traj.y, traj.dir
                )

            # if the intersection doesn't occur or if it occurs in the
            #  opposite direction, return False
            if False == s:
                return False

            intersection = Point(
                traj.x + s*math.cos(traj.dir),
                traj.y + s*math.sin(traj.dir)
                )

            # if the intersection occured inside the circle, check the real
            #  intersection on the circle's perimeter
            a = GLib.euclid(intersection.x, intersection.y, circle.x, circle.y)

            if a < circle.r:
                r = circle.r
                b = math.sqrt(r*r - a*a)
                dist = GLib.euclid(intersection.x, intersection.y, traj.x, traj.y)

                if s < 0:
                    dist *= -1

                if dist - b > 0:
                    dist -= b
                elif dist + b > 0:
                    dist += b
                else:
                    return False

                return TrajIntersection(
                    Point(
                        traj.x + dist*math.cos(traj.dir),
                        traj.y + dist*math.sin(traj.dir)
                        ),
                    dist
                    )

        return False

    @staticmethod
    def calcIntersection(x, y, theta, linear, angular, circs):
        pose = Pose(
            x,
            y,
            theta,
            linear,
            angular
            )

        traj = CLib.calculateTrajetory(pose)

        closest = TrajIntersection(
            False,
            LARGE_DISTANCE
            )

        for circ in circs:
            intersection = CLib.trajectoryIntersection(pose, traj, circ)

            if intersection:
                if intersection.delta < closest.delta:
                    closest = intersection

        closest.traj = traj

        return closest

    @staticmethod
    def calcClearance(x, y, theta, linear, angular, circs):
        res = CLib.calcIntersection(x, y, theta, linear, angular, circs)

        if res.point:
            return res.delta

        return False

print "CLib compiled!"
