#!/usr/bin/env python
#
# Uses error estimates to create, propagate, and store a collection of
# location estimates.
#
# TODO: The error estimates should be included as a matrix, so that theta
# errors can be proportional to changes in x and y, and vice versa.
#
import roslib ; roslib.load_manifest('odom_filter')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance, Point, Quaternion, Vector3
from perfesser.msg import Announce, Pt, Belief
from perfesser.srv import Guess, GuessRequest, GuessResponse
from irobot_create_2_1a.msg import SensorPacket
import math
import numpy as np
import sys
# debug only
from std_msgs.msg import String

class OdomMsmts(object):
    """
    Holds a collection of Odometry objects, indexed by their time stamp.
    """
    def __init__(self):
        # Holds a floating-point rendition of the time stamps for each Odometry.
        # We keep this in its own list because the order is important.
        self.timeList = []
        # Contains the Odometry objects themselves, indexed by the time stamp
        # floating-point numbers in self.timeList.
        self.msmts = {}

    def addMsmt(self, odom):
        """
        Adds an odometry measurement to the stack.
        """
        tkey = odom.header.stamp.to_sec()

        if self.timeList:
            assert tkey > self.timeList[-1]

        if not tkey in self.timeList:
            self.timeList.append(tkey)
            self.msmts[tkey] = odom

    def discardPrev(self, nowStamp):
        """
        Discards measurements made before the input time stamp.
        """
        tNow = nowStamp.to_sec()

        t = self.timeList[0]
        while t < tNow:
            self.timeList.pop(0)
            del self.msmts[t]
            t = self.timeList[0]

    def interpOdom(self, o1, o2, frac):
        """
        Interpolates two odometry objects, and returns a third one based
        on frac, the distance between the two.
        """
        pos1 = o1.pose.pose.position
        pos2 = o2.pose.pose.position
        posOut = Point(x = pos1.x + (pos2.x - pos1.x) * frac,
                       y = pos1.y + (pos2.y - pos1.y) * frac,
                       z = pos1.z + (pos2.z - pos1.z) * frac)
        or1 = o1.pose.pose.orientation
        or2 = o2.pose.pose.orientation
        orOut = Quaternion()

        # Interpolating quaternions is complicated.  First,
        # compute the dot product of the quaternions.  "Theta" in what
        # follows is the angle between the two quaternions.
        cosHalfTheta = or1.z * or2.z + or1.w * or2.w

        if math.fabs(cosHalfTheta) >= 1.0:
            orOut.z = or1.z
            orOut.w = or1.w
        else:
            halfTheta = math.acos(cosHalfTheta)
            sinHalfTheta = (1.0 - cosHalfTheta**2)**0.5
            if math.fabs(sinHalfTheta) < 0.001:
                orOut.z = (or1.z + or2.z)/2.0
                orOut.w = (or1.w + or2.w)/2.0
            else:
                rA  = math.sin((1 - frac) * halfTheta) / sinHalfTheta
                rB  = math.sin(frac * halfTheta) / sinHalfTheta

                orOut.w = (or1.w * rA + or2.w * rB);
                orOut.z = (or1.z * rA + or2.z * rB);

        poseOut = Pose(position = posOut, orientation = orOut)
        poseCVOut = PoseWithCovariance(pose = poseOut)

        lin1 = o1.twist.twist.linear
        lin2 = o2.twist.twist.linear
        linOut = Vector3(x = lin1.x + (lin2.x - lin1.x) * frac,
                         y = lin1.y + (lin2.y - lin1.y) * frac,
                         z = lin1.z + (lin2.z - lin1.z) * frac)
        ang1 = o1.twist.twist.angular
        ang2 = o2.twist.twist.angular
        angOut = Vector3(x = ang1.x + (ang2.x - ang1.x) * frac,
                         y = ang1.y + (ang2.y - ang1.y) * frac,
                         z = ang1.z + (ang2.z - ang1.z) * frac)
        twistOut = Twist(linear = linOut, angular = angOut)
        twistCVOut = TwistWithCovariance(twist = twistOut)

        return Odometry(pose = poseCVOut, twist = twistCVOut)

    def fromTime(self, fromStamp):
        """
        Returns a collection of measurements taken since the input time.
        The first point is estimated.  Given a situation like this:
          t0--------t1-------t2-------t3----...
                tr
        The return is [tr, t1, t2, t3,... ], where tr is interpolated
        between t0 and t1.
        """
        tFrom = fromStamp.to_sec()

        firstAfter = 0 ; l = len(self.timeList)
        if l == 0:
            frac = 0.0
            tAfter = self.timeList[0]
            tBefore = self.timeList[0]
        else:
            while ((firstAfter < l) and \
                       (self.timeList[firstAfter] < tFrom)):
                firstAfter += 1

            # There are four cases here.  Either tFrom precedes the
            # entire list, or it falls within the list, or it follows the
            # entire list (and the list is either one element or more).
            if firstAfter == 0:
                # tFrom precedes the list.
                tAfter = self.timeList[firstAfter + 1]
                tBefore = self.timeList[firstAfter]
                frac = (tFrom - tBefore) / (tAfter - tBefore)

            elif firstAfter == len(self.timeList):
                # list precedes tFrom
                if len(self.timeList) > 1:
                    tAfter = self.timeList[firstAfter - 1]
                    tBefore = self.timeList[firstAfter - 2]
                    frac = (tFrom - tBefore) / (tAfter  - tBefore)
                else:
                    tAfter = self.timeList[firstAfter - 1]
                    tBefore = tAfter
                    frac = 0.0
            else:
                # tFrom is within the list bounds.
                tAfter = self.timeList[firstAfter]
                tBefore = self.timeList[firstAfter - 1]
                if tAfter == tBefore :
                    frac = 0.0
                else:
                    frac = (tFrom - tBefore) / (tAfter - tBefore)

        oAfter = self.msmts[tAfter]
        oBefore = self.msmts[tBefore]
        oOut = self.interpOdom(oBefore, oAfter, frac)

        oOut.header.stamp = fromStamp

        # if frac == zero, then we don't need the first point.
        outList = [oOut] if frac != 0.0 else []
        for t in self.timeList[firstAfter:]:
            outList.append(self.msmts[t])

        return outList

    def formatOdometry(self, o):
        """
        Returns a nicely formatted odometry object's data.
        """
        out = "time: %d:%d\n" % (o.header.stamp.secs, o.header.stamp.nsecs)
        out += "    position.x: %7.3f           orientation.x: %7.3f\n" % \
            (o.pose.pose.position.x,o.pose.pose.orientation.x)
        out += "            .y: %7.3f                      .y: %7.3f\n" % \
            (o.pose.pose.position.y,o.pose.pose.orientation.y)
        out += "            .z: %7.3f                      .z: %7.3f\n" % \
            (o.pose.pose.position.z,o.pose.pose.orientation.z)
        out += "                                             .w: %7.3f\n" % \
            (o.pose.pose.orientation.w)

        out += "twist.linear.x: %7.3f         twist.angular.x: %7.3f\n" % \
            (o.twist.twist.linear.x, o.twist.twist.angular.x)
        out += "            .y: %7.3f                      .y: %7.3f\n" % \
            (o.twist.twist.linear.y, o.twist.twist.angular.y)
        out += "            .z: %7.3f                      .z: %7.3f\n" % \
            (o.twist.twist.linear.z, o.twist.twist.angular.z)

        return out

    def str(self):
        """
        Returns a nicely formatted list of odometry objects.
        """
        out = ""
        for t in self.timeList:
            out += "%.3f: %s\n" % (t, self.formatOdometry(self.msmts[t]))
        return out

class OdomGuesser(object):
    """
    Use this class to manage an incoming stream of odometry data and
    use it to update an occasional set of location estimates.  The
    incoming odometry data is used to maintain a relative position and
    the relative position is used to update the absolute position
    represented by the location belief.
    """
    def __init__(self, name, nPoints,
                 posErrorMag, thetaErrorMag, phiErrorMag,
                 pub=False, announce=False, debug=False):
        # This is a list of odometry measurements.  When a request comes in,
        # we'll use this list to generate an educated guess about our position.
        self.msmts = OdomMsmts()
        # The expected error for the travel from the old coordinates
        # to here.  This is the error in the linear distance, and is
        # proportional to the distance.  Units are error per unit of
        # distance.
        self.posErrorMag = posErrorMag
        # This is an error in the turning, proportional to the angle
        # turned.  Units are radians.
        self.thetaErrorMag = thetaErrorMag
        # This is the direction error suffered while traveling in a
        # straight line.  Units are radians per distance unit.  Notice
        # that we are assuming this error to be small, or else dr
        # would be affected, too.
        self.phiErrorMag = phiErrorMag

        # This is a factor by which to increase the error estimates when
        # a bump is detected.  (Odometry deteriorates on most bumps.)
        self.bumpErr = 1.0

        self.nPoints = nPoints
        self.data_type = "position"
        self.name = name

        # The publisher (if any)
        self.pub = pub

        # The publisher of the Announce() objects.  This is how the
        # odom_filter notifies the perfesser node that it has odometry
        # data to apply to the location estimates.
        self.announce = announce
        self.announced = False

        self.lastUpdated = rospy.Time()

        # This is used to record the time stamp of the incoming odom
        # data.
        self.stamp = rospy.Time()

        # This is a string to be included in the messages to the
        # perfesser.  Think of it as recording the provenance of the
        # location estimate.
        self.source_data = ""

        self.ready_to_publish = False
        self.debug = debug
        if self.debug:
            self.pdebug = rospy.Publisher("guesser/position/debug",String)

    def fixAngle(self, angle):
        """
        Ensures an angle between -pi and pi.
        """
        fixed = angle % (2 * math.pi)
        if fixed > math.pi:
            fixed -= (2 * math.pi)
        return fixed

    def updateOdom(self, odom):
        """
        This is the callback listening to odom messages.  We receive the
        message, park the relevant data into our list of odometry objects,
        and await a request to use it in updateBelief().
        """
        if self.debug:
            print "updateOdom:", self.msmts.formatOdometry(odom)

        self.msmts.addMsmt(odom)

        # Discard any measurements taken more than a few minutes ago.
        stamp = rospy.Time()
        stamp.secs = odom.header.stamp.secs - 120
        stamp.nsecs = odom.header.stamp.nsecs
        self.msmts.discardPrev(stamp)

        # Are we announcing our progress?
        if self.announce:
            now = rospy.Time.now()
            # If so, have we done so already?
            if self.announced:
                # Yes, has it been more than a half second since we updated?
                # If so, we're ready to announce again.
                if now.to_sec() - self.lastUpdated.to_sec() > 0.5:
                    self.announced = False
            else:
                # No, then announce away.
                self.announce.publish(Announce(sender=self.name))
                self.announced = True



    def propagateError(self, inPts, startOdom, endOdom):
        """
        Using the error magnitudes set at object initialization,
        estimates the position change and error accumulated in a move
        from startOdom to endOdom.  The estimate is added to the input
        errors array, and the result is returned.

        Some of this function is specific to the iRobot Creates and the
        brown-ros-pkg driver for them.  See comments below.
        """
        assert isinstance(inPts, list)
        assert isinstance(inPts[0], Pt)

        dX = endOdom.pose.pose.position.x - startOdom.pose.pose.position.x
        dY = endOdom.pose.pose.position.y - startOdom.pose.pose.position.y

        # Quaternions in the Odometry message are not used to describe
        # a rotation (which would be the natural way to use them), but
        # to describe a specific orientation.  So convert them into
        # simple angles and forego all the advantages they would have
        # provided.  Oh, well.

        # Here's something really stupid.  When Python converts a float
        # to whatever ROS publishes, and then converts it back, there is
        # sometimes an error in the fourteenth decimal place.  Tiny, but
        # enough to cause a domain error in the asin() function.
        sz = startOdom.pose.pose.orientation.z
        sz = math.copysign(min(1.0, math.fabs(sz)), sz)
        ez = endOdom.pose.pose.orientation.z
        ez = math.copysign(min(1.0, math.fabs(ez)), ez)

        startTh = math.copysign(2 * math.asin(sz),
                                sz * startOdom.pose.pose.orientation.w)
        endTh = math.copysign(2 * math.asin(ez),
                              ez * endOdom.pose.pose.orientation.w)
        dTh = self.fixAngle(endTh - startTh)

        # The heading isn't important, since we'll use the heading of the
        # robot's last reported position.  But we need the distance to
        # calculate the errors.  What is important is whether the robot is
        # moving forwards or backwards, so there is a sign involved.
        dR = math.hypot(dX, dY)
        phi = math.atan2(dY, dX)

        # If the motion is more than pi/2 away from the orientation,
        # we're moving backwards. so reverse the sign of dR.
        # n.b. What is going on here is a sneaky elision of the lack
        # of knowledge of the robot's own reference frame and the
        # transform to convert that frame into the world frame.  This
        # works, but is specific to robots who can only move forward
        # and backward.  A robot with a wider range of motion will
        # have to find a more sophisticated way to fudge the lack of
        # knowledge (or increase the error estimates).
        if 0.5 < math.fabs(phi - endTh)/math.pi < 1.5:
            dR = -dR

        outPts = []
        for pt in inPts:
            # The errors are proportional to the distance traveled and rotated.
            dRE = dR * np.random.normal(0.0, self.posErrorMag * self.bumpErr)
            dPhE = dR * np.random.normal(0.0, self.phiErrorMag * self.bumpErr)
            dThE = dTh * np.random.normal(0.0, self.thetaErrorMag * self.bumpErr)

            th = pt.point[2]
            dRx = (dR + dRE) * math.cos(th)
            dRy = (dR + dRE) * math.sin(th)

            pto = Pt(point = [ \
                    pt.point[0] + dRx * math.cos(th) - dRy * math.sin(th),
                    pt.point[1] + dRx * math.sin(th) + dRy * math.cos(th),
                    self.fixAngle(pt.point[2] + dTh + dThE + dPhE) ] )

            outPts.append(pto)

        self.bumpErr = 1.0
        return outPts

    def trackBumps(self, req):
        """
        Increases the estimate of our error if there's a bump.
        """
        if req.bumpLeft or req.bumpRight:
            self.bumpErr = 100.0

    def tidyPrint(self, odom):
        """
        Returns a compact string representation of an odometry object.
        """
        oz = odom.pose.pose.orientation.z
        sz = math.copysign(min(1.0, math.fabs(oz)), oz)
        th = math.copysign(2 * math.asin(sz),
                           oz * odom.pose.pose.orientation.w)
        return "odom, %.4f, %.4f, %.4f" % \
            (odom.pose.pose.position.x, odom.pose.pose.position.y, th)

    def updateBelief(self, req):
        """
        Takes a 'belief' object containing a set of points with a time
        stamp that indicate a location at some time in the past, and
        combines them with the Odometry messages received since then
        to formulate a new belief about where we are now.
        """
        assert isinstance(req, GuessRequest)
        reply = GuessResponse()

        # Get the odometry messages since the source time recorded in
        # the request.  More will be recorded soon.
        odoms = self.msmts.fromTime(req.source_stamp)

        # If there aren't enough odometry messages recorded, skip it.
        if len(odoms) < 2:
            reply.no_data = True
            return reply

        # Create a position array of zeros.  The odometry messages
        # record what has happened to a position since the given time.
        # We will apply those changes to these zeros, and then add the
        # result to the input points.
        pts = [ Pt(point = [0.0, 0.0, 0.0]) for i in range(self.nPoints) ]

        # Add up the position, plus error, since then.
        for start, finish in zip(odoms, odoms[1:]):
            pts = self.propagateError(pts, start, finish)
            reply.source_data += "\n%s -> %s (%.4f -- %d)" % \
                (self.tidyPrint(start), self.tidyPrint(finish), \
                     finish.header.stamp.to_sec(), finish.header.seq)

        reply.outPoints = []
        # Add the errors to the input points.  These are in different
        # reference frames, so the addition is not simple.  We convert
        # the adjustment to polar coordinates and apply it as a relative
        # adjustment to the input points.
        #
        # We're also adding a little wiggle to simulate annealing.
        wiggle = lambda x: 1.0 * x * ((2 * np.random.random()) - 1)
        # A bit more about the wiggle: a belief is a collection of
        # individual points, but is intended to be a representation of
        # a probability distribution.  Without the wiggle, over time
        # the distribution evolves into a collection of individual
        # points, and drifts away from being a decent representation
        # of a distribution.  The wiggling helps keep the definition
        # clean, and it also helps adjust to sudden changes, like when
        # someone picks up the robot and moves it.

        for pt, inpt in zip(pts, req.inPoints):
            dR = math.hypot(pt.point[0], pt.point[1])
            dPh = math.atan2(pt.point[1], pt.point[0])
            dTh = pt.point[2]

            dX = dR * math.cos(inpt.point[2] + dPh)
            dY = dR * math.sin(inpt.point[2] + dPh)

            outpt = Pt(point= [ inpt.point[0] + dX + wiggle(self.posErrorMag),
                                inpt.point[1] + dY + wiggle(self.posErrorMag),
                                self.fixAngle(inpt.point[2] + dTh + \
                                                  wiggle(self.thetaErrorMag)) ])

            reply.outPoints.append(outpt)

        # Reply.
        reply.no_data = False
        reply.sender = self.name
        last = odoms[-1]
        reply.source_stamp = last.header.stamp
        self.pdebug.publish(reply.source_data)

        self.lastUpdated = last.header.stamp

        return reply


#######################

if __name__ == "__main__":

    rospy.init_node("odom_filter")
    print "************MUST HAVE ROSCORE RUNNING SOMEWHERE********"


    npts = 10
    o = OdomGuesser("testOdomGuesser", npts, 0.001, 0.001, 0.001, debug=True)

    init = Odometry()
    th = 0.0
    init.pose.pose.orientation.z = math.sin(th/2.0)
    init.pose.pose.orientation.w = math.cos(th/2.0)
    init.header.stamp = rospy.Time.from_sec(1310140130.951)
    o.msmts.addMsmt(init)

    ol = [init]

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.0
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.012)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.001
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.088)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.007
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.161)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.013
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.224)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.020
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.296)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.028
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.372)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.034
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.436)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.043
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.507)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.052
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.584)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    th = 0.0
    ol[-1].pose.pose.position.x = 0.058
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310140131.643)
    o.msmts.addMsmt(ol[-1])

    # Arrange empty position array
    pts = [ Pt(point = [0.0, 0.0, 0.0]) for i in range(npts) ]

    print o.propagateError(pts, init, ol[-1])

    greq = GuessRequest()
    greq.source_stamp = rospy.Time(secs=init.header.stamp.secs,nsecs=init.header.stamp.nsecs)
#    greq.source_stamp.nsecs -= 10000
    greq.inPoints = pts

    output = o.updateBelief(greq)
    print output

    from perfesser_guesser import Guesser
    g = Guesser("testGuesser", (0.0,0.0,2*math.pi), "location",
                (4,4,7))
    g.newPoints(output.outPoints)
    m = g.means()

    print m

    assert math.fabs(m[0] - 0.058) < 0.001
    assert math.fabs(m[1]) < 0.001
    assert math.fabs(m[2]) < 0.001



    o1 = Odometry()
    th = 0.0
    o1.pose.pose.position.x = 1.0
    o1.pose.pose.orientation.z = math.sin(th/2.0)
    o1.pose.pose.orientation.w = math.cos(th/2.0)

    o2 = Odometry()
    th = -0.25
    o2.pose.pose.position.x = 1.0
    o2.pose.pose.orientation.z = math.sin(th/2.0)
    o2.pose.pose.orientation.w = math.cos(th/2.0)

    oOut = o.msmts.interpOdom(o1, o2, -0.5)

    th = math.copysign(2 * math.asin(oOut.pose.pose.orientation.z), \
                           oOut.pose.pose.orientation.z * \
                               oOut.pose.pose.orientation.w)

    print o.msmts.formatOdometry(o1)
    print o.msmts.formatOdometry(o2)
    print th, o.msmts.formatOdometry(oOut)

    o = OdomGuesser("testOdomGuesser", npts, 0.0001, 0.0001, 0.0001, debug=True)

    ol = []

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.31363
    ol[-1].pose.pose.position.y = 0.00168
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.05915)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.31478
    ol[-1].pose.pose.position.y = 0.00004
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.13612)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.31764
    ol[-1].pose.pose.position.y = -0.00405
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.19581)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.32108
    ol[-1].pose.pose.position.y = -0.00897
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.27032)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.32510
    ol[-1].pose.pose.position.y = -0.01470
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.34477)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.32854
    ol[-1].pose.pose.position.y = -0.01962
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.40470)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.33313
    ol[-1].pose.pose.position.y = -0.02617
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.47988)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.33829
    ol[-1].pose.pose.position.y = -0.03354
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.55567)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.34173
    ol[-1].pose.pose.position.y = -0.03846
    th = -0.95993
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    ol[-1].header.stamp = rospy.Time.from_sec(1310172426.61597)
    o.msmts.addMsmt(ol[-1])

    # Arrange empty position array
    pts = [ Pt(point = [0.31345, 0.00124, -0.959896]) for i in range(npts) ]

    print o.propagateError(pts, ol[0], ol[-1])

    greq = GuessRequest()
    greq.source_stamp = rospy.Time(secs=ol[0].header.stamp.secs,nsecs=ol[0].header.stamp.nsecs)
#    greq.source_stamp.nsecs -= 10000
    greq.inPoints = pts

    output = o.updateBelief(greq)
    print output

    from perfesser_guesser import Guesser
    g = Guesser("testGuesser", (0.0,0.0,2*math.pi), "location",
                (4,4,7))
    g.newPoints(output.outPoints)
    m = g.means()
    print m
    print "********************************************"

    o = OdomGuesser("testOdomGuesser", npts, 0.0001, 0.0001, 0.0001, debug=True)
    ol = []

    # ol.append(Odometry())
    # ol[-1].pose.pose.position.x = 0.0759
    # ol[-1].pose.pose.position.y = -0.1182
    # th = -2.8798
    # ol[-1].header.stamp = rospy.Time.from_sec(1310584995.8524)
    # ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    # ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    # o.msmts.addMsmt(ol[-1])

    # ol.append(Odometry())
    # ol[-1].pose.pose.position.x = 0.0682
    # ol[-1].pose.pose.position.y = -0.1203
    # th = -2.8798
    # ol[-1].header.stamp = rospy.Time.from_sec(1310584995.9246)
    # ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    # ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    # o.msmts.addMsmt(ol[-1])

    # ol.append(Odometry())
    # ol[-1].pose.pose.position.x = 0.0605
    # ol[-1].pose.pose.position.y = -0.1223
    # th = -2.8798
    # ol[-1].header.stamp = rospy.Time.from_sec(1310584996.0006)
    # ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    # ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    # o.msmts.addMsmt(ol[-1])

    # ol.append(Odometry())
    # ol[-1].pose.pose.position.x = 0.0547
    # ol[-1].pose.pose.position.y = -0.1239
    # th = -2.8798
    # ol[-1].header.stamp = rospy.Time.from_sec(1310584996.0606)
    # ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    # ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    # o.msmts.addMsmt(ol[-1])

    # ol.append(Odometry())
    # ol[-1].pose.pose.position.x = 0.0470
    # ol[-1].pose.pose.position.y = -0.1260
    # th = -2.8798
    # ol[-1].header.stamp = rospy.Time.from_sec(1310584996.1368)
    # ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    # ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    # o.msmts.addMsmt(ol[-1])

    # ol.append(Odometry())
    # ol[-1].pose.pose.position.x = 0.0383
    # ol[-1].pose.pose.position.y = -0.1283
    # th = -2.8798
    # ol[-1].header.stamp = rospy.Time.from_sec(1310584996.2126)
    # ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    # ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    # o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.0325
    ol[-1].pose.pose.position.y = -0.1299
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.2726)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.0238
    ol[-1].pose.pose.position.y = -0.1322
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.3486)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.0151
    ol[-1].pose.pose.position.y = -0.1345
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.4206)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.0083
    ol[-1].pose.pose.position.y = -0.1363
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.4847)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = 0.0006
    ol[-1].pose.pose.position.y = -0.1384
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.5566)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = -0.0081
    ol[-1].pose.pose.position.y = -0.1407
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.6345)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])

    ol.append(Odometry())
    ol[-1].pose.pose.position.x = -0.0235
    ol[-1].pose.pose.position.y = -0.1449
    th = -2.8798
    ol[-1].header.stamp = rospy.Time.from_sec(1310584996.7681)
    ol[-1].pose.pose.orientation.z = math.sin(th/2.0)
    ol[-1].pose.pose.orientation.w = math.cos(th/2.0)
    o.msmts.addMsmt(ol[-1])


    # Arrange empty position array
    pts = [ Pt(point = [0.032034, -0.1299, -2.8798]) for i in range(npts) ]
    print o.propagateError(pts, ol[0], ol[-1])

    greq = GuessRequest()
    greq.source_stamp = rospy.Time(secs=ol[0].header.stamp.secs,nsecs=ol[0].header.stamp.nsecs)
    greq.inPoints = pts

    output = o.updateBelief(greq)
    print output

    from perfesser_guesser import Guesser
    g = Guesser("testGuesser", (0.0,0.0,2*math.pi), "location",
                (4,4,7))
    g.newPoints(output.outPoints)
    m = g.means()
    print m





    import sys
    if len(sys.argv) > 1:

        def odogenerator(x):
            out = Odometry()
            out.pose.pose.position.x = x + 0.011
            out.pose.pose.position.y = x + 0.012
            out.pose.pose.position.z = x + 0.013

            out.pose.pose.orientation.x = 0.0
            out.pose.pose.orientation.y = 0.0
            th = 0.25 * math.pi / x
            out.pose.pose.orientation.z = math.sin(th/2.0)
            out.pose.pose.orientation.w = math.cos(th/2.0)

            out.twist.twist.linear.x = x + 0.021
            out.twist.twist.linear.y = x + 0.022
            out.twist.twist.linear.z = x + 0.023

            out.twist.twist.angular.x = x + 0.024
            out.twist.twist.angular.y = x + 0.025
            out.twist.twist.angular.z = x + 0.026

            out.header.stamp = rospy.Time.now()

            return out


        o1 = odogenerator(0.1)
        rospy.sleep(0.6)
        o2 = odogenerator(1.2)
        rospy.sleep(0.6)
        o3 = odogenerator(0.3)
        rospy.sleep(0.6)
        o4 = odogenerator(0.4)

        oms = OdomMsmts()

        oms.addMsmt(o1)
        oms.addMsmt(o2)
        oms.addMsmt(o3)
        oms.addMsmt(o4)

        s = rospy.Time.now()
        s.secs -= 1
        s.nsecs -= 500000000
        if s.nsecs < 0:
            s.secs -= 1
            s.nsecs += 1000000000

        om = oms.fromTime(s)

#    print oms.formatOdometry(om[0])

        oms.discardPrev(s)
        ol = oms.fromTime(s)

#    print oms.formatOdometry(ol[0])

        o = OdomGuesser("testOdomGuesser", 1000, 0.1, 0.1, 0.1, debug=True)

        import tgraph
        from perfesser_guesser import *

        g = Guesser("testGuesser", (0.0, 0.0, 2*math.pi),
                    "position", (4,4,7))
        g.uniform(1000, [[-2.0,2.0], [-2.0,2.0], [0.0, 0.5]])

        tg = tgraph.Tgraph(400,220)
        corners = np.array([[-4.0, -4.0, -4.0], [4.0, 4.0, 4.0]])
        tg.draw_scatter(corners, 0, 1, 2, "s")
        tg.plot_points(g.pointArray, 0, 1, 2, "s")


        o.updateOdom(o1)
        o.updateOdom(o2)
        o.updateOdom(o3)
        o.updateOdom(o4)

        s = o1.header.stamp
        s.nsecs += 5000000

        greq = GuessRequest()
        greq.source_stamp = s
        greq.header.stamp = rospy.Time.now()
        greq.pers = (0.0, 0.0, 2*math.pi)
        greq.means = g.means()
        greq.stds = g.stds()
        greq.inPoints = g.outPoints()

        gresp = o.updateBelief(greq)

        pts = Belief(points = gresp.outPoints)
        tg.new_graph()
        pta = g.pointsToArray(pts)
        print "maxs:", map(max, pta.transpose())
        print "mins:", map(min, pta.transpose())

        tg.draw_scatter(g.pointsToArray(pts), 0, 1, 2, "s", recalc=False)

        tg.mainloop()
