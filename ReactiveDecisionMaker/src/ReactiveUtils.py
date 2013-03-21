import math

pi2 = math.pi*2

class ReactiveUtils:
    # these constants are in meters
    MAX_VAL = 1 
    CLOSE_ENOUGH_TO_GOAL = .1
    MIN_CLEARANCE = 1

    @staticmethod
    def boundAngleTo2PI(angle):
        return (angle%pi2 + pi2)%pi2;

    @staticmethod
    def boundAngleToNPItoPI(angle):
        angle = ReactiveUtils.boundAngleTo2PI(angle);
        if angle > math.pi:
            angle -= pi2
        return angle

    @staticmethod
    def euclidDistPoint(p0, p1):
        return math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

    @staticmethod
    def euclidDistArr(p0, p1):
        return math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)

    @staticmethod
    def angle_dif(a1, a2):
        a1 = ReactiveUtils.boundAngleTo2PI(a1)
        a2 = ReactiveUtils.boundAngleTo2PI(a2)
        r = ReactiveUtils.boundAngleTo2PI(a1 - a2)

        if r > math.pi:
            r -= pi2

        return r

    @staticmethod
    def calcClearance(curPos, angle1, dist1, angle2, dist2) :
        dist = min(dist1, dist2)

        p0 = (curPos.x + dist*math.cos(angle1),\
	          curPos.y + dist*math.sin(angle1))
        p1 = (curPos.x + dist*math.cos(angle2),\
	          curPos.y + dist*math.sin(angle2))

        return ReactiveUtils.euclidDistArr(p0, p1)

    @staticmethod	
    def averageAngle(angle1, angle2):
        xcomp = (math.cos(angle1) + math.cos(angle2))/2.0
        ycomp = (math.sin(angle1) + math.sin(angle2))/2.0
	
        return math.atan2(ycomp, xcomp)
