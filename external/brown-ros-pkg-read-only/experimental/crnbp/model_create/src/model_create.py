#!/usr/bin/env python
# This is a rudimentary modeler for irobot creates.  It has a model of
# a room, and supports multiple robots within that room.  Each robot
# has a twist() method that can be used to subscribe to cmd_vel
# messages and move the robot avatar around the room accordingly.  It
# also broadcasts odom messages for those who are interested in them.
# The sensor packet is not implemented (yet).
#
# To run it, create a RoomModel, and then create robots with
# CreateModel and add them to the room.  Calling
# RoomModel.updateRoom() will advance the position of each robot in
# the room according to its own current velocities.
#
# You can execute this file directly, which will draw a small room and
# a single robot in it, which can then be manipulated with cmd_vel
# (Twist) messages.
#
# There is a model of how to do this in the cover package.
#
import roslib ; roslib.load_manifest('model_create')
import rospy
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from irobot_create_2_1a.msg import SensorPacket
from irobot_create_2_1a.srv import *
import math
import tgraph
import force
import numpy as np

class RoomModel(object):
    def __init__(self, dt, xdim, ydim, xscale = 1.0, yscale = 1.0, 
                 xmin=-1e35, ymin=-1e35, debug=False):
        self.xdim = xdim
        self.ydim = ydim
        self.xscale = xscale
        self.yscale = yscale
        self.xmin = xmin
        self.ymin = ymin
        self.robots = {}
        self.dt = dt
        self.time = 0.0
        self.tg = tgraph.Tgraph(xdim, ydim)

        if self.xmin < -1.0e34 :
            corners = np.array([[-self.xscale, -self.yscale,0.0],
                                [self.xscale, self.yscale, 100.0]])
        else:
            corners = np.array([[self.xmin, self.ymin,0.0],
                                [self.xscale, self.yscale, 100.0]])

        self.tg.draw_scatter(corners, 0,1,2,"s")
        self.tg.w[-1].update()
        self.debug = debug
        # List for keeping the tk ids of the target, so we can delete items 
        # easily.
        self.targetList = []

        self.paused = False

    def addRobot(self, robot):
        self.robots[robot.name] = robot
        robot.draw_pos(self.tg)

    def delRobot(self, name):
        try:
            self.robots[name].remove(self.tg)
            del self.robots[name]
            print "REMOVED ", name
            return True
        except KeyError:
            return False

    def pause(self, req):
        """
        Toggle pause.
        """
        if self.paused:
            self.paused = False
        else:
            self.paused = True

    def updateHook(self):
        return

    def updateRoom(self):

        if self.paused:
            return

        self.time += self.dt

        if not self.robots:
            return

        for robot in self.robots.values():
            # clear canvas?
            robot.advance(self.dt)
            robot.draw_pos(self.tg)
            if self.debug:
                print "Robot(%s): (%5.3f,%5.3f,%5.3f) going (%5.3f,%5.3f)" % \
                    (robot.name, robot.x, robot.y, robot.th, \
                     robot.curTwist.linear.x, robot.curTwist.angular.z)

        self.updateHook()

    def drawLine(self, start, end):
        return self.tg.w[self.tg.cur].create_line(start[0], start[1], end[0], end[1])

    def placeTarget(self, req):
        if self.targetList:
            for piece in self.targetList:
                self.tg.w[self.tg.cur].delete(piece)

        pt = self.tg.screen_coords(req.position.x, req.position.y)

        self.targetList.append(\
            self.tg.w[self.tg.cur].create_line(pt[0] - 5 * self.tg.tick, pt[1],
                                               pt[0] + 5 * self.tg.tick, pt[1], 
                                               fill="red"))
        self.targetList.append(\
            self.tg.w[self.tg.cur].create_line(pt[0], pt[1] - 5 * self.tg.tick,
                                               pt[0], pt[1] + 5 * self.tg.tick, 
                                               fill="red"))
        self.targetList.append(\
            self.tg.w[self.tg.cur].create_oval(pt[0] - 2, pt[1] - 2,
                                               pt[0] + 2, pt[1] + 2, 
                                               fill="red"))



class CreateModel(object):
    """
    Class to instantiate a model of an irobot Create.  The robot attempts to
    model the steering and speeds of the real item.
    """
    def __init__(self, x, y, th, name="", color=2, trails=False, 
                 nose=True, radius = 0.13, debug=False):
        self.x = x
        self.y = y
        self.th = th
        self.forceList = force.ForceList(0.11,0.5,0.5,1.0)
        self.curTwist = Twist()
        self.then = 0.0
        self.radius = radius
        self.color = color
        self.debug = debug
        self.name = name
        self.nose = nose

        self.lwSpeed = 0.0 ; self.rwSpeed = 0.0
        self.distanceBetweenWheels = .260 # mm (CONSTANT)
        self.wheelDistanceFromCenter = self.distanceBetweenWheels / 2.0
        self.then = 0.0

        topic = (self.name + "/") if self.name else ""

        self.odomPub = rospy.Publisher(topic + 'odom', Odometry)
        self.odomBroadcaster = TransformBroadcaster()

        self.sensorPub = rospy.Publisher(topic + 'sensors', SensorPacket)

        self.trails = trails # If true, redraws each time.  False moves without trails.
        self.bodyObj = 0 ; self.noseObj = 0 ; self.leftObj = 0 ; self.rightObj = 0
        self.oldScreenPts = [[0.0,0.0]*4]

        self.tg = []

    def detectSomething(self, sensor):
        self.sensorPub.publish(sensor)

    def tank(self,left,right):
        """
        Drive the iCreate like a tank (i.e. left throttle, right
        throttle). Takes two parameters: left and right throttle. Each
        can be between -500 and 500 representing mm/s.  If either are
        outside of this range, both throttles will be linearly scaled
        to fit inside the range.
        """
        if abs(left) > 500 or abs(right) > 500:
            maxThrottle = max(abs(left),abs(right))
            left = 500*left/maxThrottle
            right = 500*right/maxThrottle
        self.lwSpeed = left * 0.001
        self.rwSpeed = right * 0.001

    def remove(self, tg):
        """
        Deletes a robot from the picture.
        """
        tg.w[tg.cur].delete(self.bodyObj)
        if self.nose:
            tg.w[tg.cur].delete(self.noseObj)
            tg.w[tg.cur].delete(self.rightObj)
            tg.w[tg.cur].delete(self.leftObj)

    def leaveMark(self):
        self.tg.plot_point_circle([self.x + 0.5 * self.radius * math.cos(self.th),
                                   self.y + 0.5 * self.radius * math.sin(self.th),
                                   2], 0, 1, 2, colval=self.color)


    # Draws the robot in its current position on tgraph object tg.
    def draw_pos(self, tg):
        self.tg = tg
        if self.debug:
            print "position: %5.3f, %5.3f, %5.3f" % (self.x, self.y, self.th)

        if self.trails:
            tg.plot_point_circle([self.x, self.y, 4],
                                 0,1,2, colval=self.color)
            if self.nose:
                tg.plot_point_circle([self.x + self.radius * math.cos(self.th),
                                      self.y + self.radius * math.sin(self.th),
                                      2], 0, 1, 2, colval=self.color)
                tg.plot_point_circle([self.x + self.radius * math.sin(self.th),
                                      self.y - self.radius * math.cos(self.th),
                                      2], 0, 1, 2, colval=self.color)
                tg.plot_point_circle([self.x - self.radius * math.sin(self.th),
                                      self.y + self.radius * math.cos(self.th),
                                      2], 0, 1, 2, colval=self.color)
        else:
            # The four entries in these lists correspond to the 
            # body, nose, left, right discs.
            pts = [[self.x, self.y]]
            pts.append([self.x + self.radius * math.cos(self.th),
                        self.y + self.radius * math.sin(self.th)])
            pts.append([self.x + self.radius * math.sin(self.th),
                        self.y - self.radius * math.cos(self.th)])
            pts.append([self.x - self.radius * math.sin(self.th),
                        self.y + self.radius * math.cos(self.th)])

            screenPts = [list(tg.screen_coords(p[0],p[1]))  for p in pts]

            if self.bodyObj:
                # We are sneakily addressing the canvas object directly here.
                tg.w[tg.cur].move(self.bodyObj, 
                                  screenPts[0][0] - self.oldScreenPts[0][0], 
                                  screenPts[0][1] - self.oldScreenPts[0][1])
                if self.nose:
                    tg.w[tg.cur].move(self.noseObj, 
                                      screenPts[1][0] - self.oldScreenPts[1][0], 
                                      screenPts[1][1] - self.oldScreenPts[1][1])
                    tg.w[tg.cur].move(self.rightObj,
                                      screenPts[2][0] - self.oldScreenPts[2][0], 
                                      screenPts[2][1] - self.oldScreenPts[2][1])
                    tg.w[tg.cur].move(self.leftObj,
                                      screenPts[3][0] - self.oldScreenPts[3][0], 
                                      screenPts[3][1] - self.oldScreenPts[3][1])
                if self.debug:
                    tg.w[tg.cur].delete(self.nline)
                    tg.w[tg.cur].delete(self.tline)
            else:
                self.bodyObj = tg.plot_point_circle([self.x, self.y, 3],
                                                    0,1,2, colval=self.color)

                if self.nose:
                    self.noseObj = tg.plot_point_circle([self.x + self.radius * math.cos(self.th),
                                                         self.y + self.radius * math.sin(self.th),
                                                         2], 0, 1, 2, colval=self.color)
                    self.rightObj = tg.plot_point_circle([self.x + self.radius * math.sin(self.th),
                                                          self.y - self.radius * math.cos(self.th),
                                                          2], 0, 1, 2, colval=self.color)
                    self.leftObj = tg.plot_point_circle([self.x - self.radius * math.sin(self.th),
                                                         self.y + self.radius * math.cos(self.th),
                                                         2], 0, 1, 2, colval=self.color)

            if self.debug:
                print "Forces: normal:%5.3f, tangent:%5.3f" % \
                    (self.curTwist.linear.y, self.curTwist.angular.y)
                self.nline = self.plot_polar(tg, [self.x, self.y], 
                                             self.curTwist.linear.y, 
                                             self.th, "green")
                self.tline = self.plot_polar(tg, [self.x,self.y], 
                                             self.curTwist.angular.y, 
                                             (self.th + (math.pi/2)), "red")

            tg.w[tg.cur].update()
            self.oldScreenPts = screenPts

        return

    def plot_polar(self, tg, pt, mag, angle, color):
        return tg.plot_lines(np.array([[pt[0], pt[1], 3.0],
                                       [pt[0] + mag * math.cos(angle),
                                        pt[1] + mag * math.sin(angle), 2.0]]),
                             0,1,2,color)

    # Advances the robot's position by time dt.
    def advance(self, dt):
        now = self.then + dt
        elapsed = now - self.then
        self.then = now
        if self.debug:
            print "advancing to t=%5.3f" % (self.then)

        ldist = dt * self.lwSpeed
        rdist = dt * self.rwSpeed

        d = (ldist + rdist)/2.0
        th = (rdist - ldist)/self.distanceBetweenWheels

        dx = d / dt
        dth = th / dt

        if (d != 0):
            x = math.cos(th)*d
            y = math.sin(th)*d # Does this differ from the create driver?  Should it?
            self.x = self.x + (math.cos(self.th)*x - math.sin(self.th)*y)
            self.y = self.y + (math.sin(self.th)*x + math.cos(self.th)*y)

        if (th != 0):
            self.th = self.th + th
            self.th = self.th % (2 * math.pi)
            self.th = self.th - (2 * math.pi) if self.th > math.pi else self.th
                

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(self.th/2.0)
        quaternion.w = math.cos(self.th/2.0)

        # Most of the applications I'm working on don't need this.
        # self.odomBroadcaster.sendTransform(
        #     (self.x, self.y, 0),
        #     (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        #     rospy.Time.now(),
        #     "base_link",
        #     "odom"
        #     )

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

    def twist(self,req):
        """
        Sets the wheel speeds, just as the twist() method of the real 
        driver does.
        """
        self.curTwist.linear.x = req.linear.x
        self.curTwist.angular.z = req.angular.z
        if self.debug:
            self.curTwist.linear.y = req.linear.y
            self.curTwist.angular.y = req.angular.y
        x = req.linear.x * 1000
        th = req.angular.z # angular speed in rad/s
        if (th == 0):
            x = int(x)
            self.tank(x,x)
            if self.debug:
                print "Forward speed:%5.3f" % (x,)
        else:
            distanceBetweenWheels = 260 # mm (CONSTANT)
            wheelDistanceFromCenter = distanceBetweenWheels / 2
            turnRadius = x/th # from perspective of robot center
            lwSpeed=int((turnRadius-wheelDistanceFromCenter)*th)
            rwSpeed=int((turnRadius+wheelDistanceFromCenter)*th)
            self.tank(lwSpeed,rwSpeed)
            if self.debug:
                print "lwSpeed:%5.3f, rwSpeed:%5.3f" % (lwSpeed/1000.0, rwSpeed/1000.0)

    def playsong(self, req):
        """
        Provides a play song service, but doesn't do anything.
        """
        print '\a\a\a\a\aWOOFWOOF'
        return PlaySongResponse(True)

    def str(self):
        """ 
        Pretty-printer for robot location.
        """
        return "%s: (%.3f,%.3f,%.3f)" % (self.name, self.x, self.y, self.th)

if __name__ == '__main__':
    # Execute this file if you want to run the robot by issuing cmd_vel messages.
    rospy.init_node("model_create")

    import time

    room = RoomModel(0.1, 500,500, 3.0, 3.0, debug=True)

    rob = CreateModel(0.0,0.0,0.0,color=2,name="rob", debug=True)
    room.addRobot(rob)

    p = Pose()
    p.position.x = 1.0
    p.position.y = 1.0
    room.placeTarget(p)

    rospy.Subscriber('/model_create/placeTarget', Pose, room.placeTarget)
    rospy.Service('play', PlaySong, rob.playsong)
    rospy.Subscriber("cmd_vel", Twist, rob.twist)


    import signal
    def handler(signum, frame):
        print "...ok, I'm stopping..."
        raise
    signal.signal(signal.SIGINT, handler)

    try:
        n = 0
        while(1):
            n += 1
            print n
            room.updateRoom()
            time.sleep(0.2)
    except:
        print "... stopped."
        pass
