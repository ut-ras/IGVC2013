#!/usr/bin/env python
#
import roslib ; roslib.load_manifest('cover')
import rospy
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
from irobot_create_2_1a.msg import SensorPacket
from model_create import CreateModel, RoomModel
from perfesser.msg import Pt
from cover.msg import Room, Room2D
from cover.srv import GetString, GetStringRequest, GetStringResponse
from room_net import RoomNet
import tgraph as tg
import math
import numpy as np
import sys

class TwoDRobotModel(CreateModel):
    def __init__(self, name, xpos, ypos, color,
                 perimeter = [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0],[0.0,0.0]],
                 radius = 0.013):
        self.xvel = 0.0
        self.yvel = 0.0
        # Perimeters must be specified clockwise (looking down).
        self.perimeter = [(p, (q[0] - p[0], q[1] - p[1])) \
                              for p,q in zip(perimeter[0:-1],perimeter[1:]) ]

        # This records the number of communication hops between here and
        # the base station or primary robot.
        self.hops = 0

        CreateModel.__init__(self, xpos, ypos, 3.0, name=name,
                             color=color, trails=False,
                             nose=True, debug=False, radius = radius)

        self.sub = rospy.Subscriber(self.name + "/cmd_vel", Twist, self.twist)

    def cross(self, p, q):
        """
        2d cross product.
        """
        return p[0] * q[1] - p[1] * q[0]

    def intersect(self, p, r, q, s):
        """
        Intersection of two line segments in 2d.  Segment one is (p, p+r) and
        two is (q, q+s).
        """
        c = self.cross(r,s)

        if c == 0:
            return (-1.0, -1.0)

        d = (q[0] - p[0], q[1] - p[1])
        t = self.cross(d, s) / c
        u = self.cross(d, r) / c

        return (t,u)


    # Advances the robot's position by time dt.  But asserts limits.
    def advance(self, dt):
        now = self.then + dt
        elapsed = now - self.then
        self.then = now

        ldist = dt * self.lwSpeed
        rdist = dt * self.rwSpeed

        d = (ldist + rdist)/2.0
        th = (rdist - ldist)/self.distanceBetweenWheels

        dx = d / dt
        dth = th / dt

        bumpLeft = False
        bumpRight = False

        if (d != 0):
            x = math.cos(th)*d
            y = math.sin(th)*d # Does this differ from the create driver?  Should it?
            newx =  (math.cos(self.th)*x - math.sin(self.th)*y)
            newy =  (math.sin(self.th)*x + math.cos(self.th)*y)
            motionDir = math.atan2(newy,newx)
            backward = True if math.fabs(motionDir - self.th) > 1.5707 else False

            # Assert the limits defined by the perimeter.
            collision = False
            # Use this to hold t and the angle of incidence.
            collisionDetail = [ 1.0, 0.0 ]
            for p in self.perimeter:
                tu = self.intersect((self.x, self.y),(newx, newy), p[0], p[1])
                if 0.0 <= tu[0] <= 1.0 and 0.0 <= tu[1] <= 1.0:
                    collision = True
                    # The segment may cross more than one perimeter segment.  The
                    # one we want has the minimum tu[0].
                    if tu[0] < collisionDetail[0]:
                        collisionDetail = [ tu[0], motionDir - \
                                     math.atan2(p[1][1],p[1][0]) ]

            if collision:
                # Normalize direction of change
                norm = (newx**2 + newy**2)**0.5

                self.x += collisionDetail[0] * newx - 0.01 * newx/norm
                self.y += collisionDetail[0] * newy - 0.01 * newy/norm

                if collisionDetail[1] < -math.pi:
                    angleIncidence = collisionDetail[1] + 2*math.pi 
                else:
                    angleIncidence = collisionDetail[1]

                # Only trigger the bump sensors if we're not moving backward.
                if not backward:
                    if 3.14159 > angleIncidence > 1.500:
                        bumpLeft = True
                    if 0.0 < angleIncidence < 1.6414:
                        bumpRight = True

                if angleIncidence < 0.0:
                    print "UH-OH", collisionDetail

            else:
                self.x += newx
                self.y += newy

        if (th != 0):
            self.th += th
            self.th = self.th % (2 * math.pi)
            self.th = self.th - (2 * math.pi) if self.th > math.pi else self.th

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(self.th/2.0)
        quaternion.w = math.cos(self.th/2.0)

        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            "base_link",
            "odom"
            )

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dth

        self.odomPub.publish(odom)

        if bumpLeft or bumpRight:
            # print self.name, "is really at:", self.x, self.y, self.th
            self.leaveMark()
            self.detectSomething(SensorPacket(bumpLeft=bumpLeft,
                                              bumpRight=bumpRight))



class TwoDRoomModel(RoomModel):
    def __init__(self, dt, xdim, neighborDistance = 3.0,
                 perimeter = [[0,0],[1,0],[1,1],[0,1],[0,0]]):
        self.dt =  dt
        self.xdim = xdim
        self.neighborDistance = neighborDistance
        self.perimeter = perimeter

        self.lines = []
        self.room2D = Room2D()

        RoomModel.__init__(self, self.dt, self.xdim, self.xdim,
                           xscale = 10.0, yscale = 10.0,
                           xmin = 0.0, ymin = 0.0)

        rospy.Service("/room/pause", GetString, self.pause)

        self.tg.plot_lines(np.array(self.perimeter),0,1,0,"purple")
    
    def nearest(self):
        """
        Returns a list of robots, with their neighbors.
        """
        net = RoomNet(self.neighborDistance,
                      [ [ robot.name, [ robot.x, robot.y ] ] \
                            for robot in self.robots.values()])
        net.connect()
        net.reciprocate()

        self.room2D = net.unwind()
        return self.room2D

    def drawConnections(self):
        if self.lines:
            for line in self.lines:
                self.tg.w[self.tg.cur].delete(line)

        self.lines = []
        for room in self.room2D.robots:
            center = self.robots[room.sourceName]
            for loc in room.locations:
                self.lines.append(\
                    self.tg.plot_lines(np.array(\
                            [[center.x,center.y,center.color],
                             [loc.point[0], loc.point[1], center.color]]),
                            0, 1, 2, "red"))

    def pause(self, req):
        """
        Toggle pause.
        """
        if self.paused:
            self.paused = False

            sout = "ok, starting up again..."

        else:
            self.paused = True

            sout = "paused at: \n"
            if self.robots:
                for robot in self.robots.values():
                    sout += robot.str() + "\n"
        out = GetStringResponse()
        out.output = sout
        return out

    def dista(self, x1, y1, x2, y2):
        """ Cartesian distance """
        return ((y2 - y1)**2 + (x2 - x1)**2)**0.5

    def dist(self, pta, ptb):
        return sum([ (a - b)**2 for a,b in zip(pta,ptb) ])**0.5

    def list(self):
        """
        Compiles a list of robots and locations in the room.
        """
        out = Room(sourceName = "listing")
        for rb in self.robots.values():
            out.names.append(rb.name)
            out.locations.append(Pt(point=(rb.x, rb.y)))
        return out


