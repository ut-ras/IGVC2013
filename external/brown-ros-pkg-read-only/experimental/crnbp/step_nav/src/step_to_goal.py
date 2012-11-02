#!/usr/bin/env python
#
# A goal consists of a set of coordinates to get to, and it exists in
# a stack of sorts.  That is, we're working towards the top goal in
# the stack, but when we get there, we have another goal right
# underneath it.  Also, if we bump into anything on the way, we might
# pre-empt the goal, either deleting it, or simply pushing another
# goal onto the stack above it.
#
# Furthermore, a goal on the stack can be modified on the fly.  That
# is, the goal point might move, for example if it depends on the
# location of its neighbors.
#
# A goal is defined in the robot's own reference frame.  Any
# translations from one coordinate system to another have to happen
# before you get to this spot.
#
# This process works toward the top goal on the stack, and also
# maintains the stack.  It will listen to the robot location,
# publishes Twist messages to get to the current top goal, and provide
# a service to push goals onto the stack and modify (or delete) goals
# already there.
#
# Service API input: goal, index (optional)  --> output: index or zero
# for failure.  So: input: (1.0, 1.0. pi),0 --> output: index = 16.
# And later (1.0, 2.0, pi), 16  --> index = 16 means success in
# redefining goal 16.
import roslib ; roslib.load_manifest('step_nav')
import rospy
from position_tracker.srv import SetPosition
from position_tracker.msg import Position
from geometry_msgs.msg import Twist
from perfesser.msg import Belief
from step_nav.msg import Fudge
import goal
import force
import math
import random

class StepToGoal(object):
    """
    Init args are the dimensions (a tuple with zeros for the
    non-periodic dimensions and the period for the others), the publisher
    for Twist messages, and robot speed control parameters.  The targetFn
    function is executed when a goal is reached.
    """
    def __init__(self, dims, publisher,
                 maxSpeed, maxTurn, linDamp, angDamp,
                 stdthres=1000.0,
                 debug=False, targetFn = False,
                 dbgdir="", fudge=0.8, signalFn=False):
        self.dims = dims
        self.pub = publisher
        self.gs = goal.GoalStack()
        self.zeroDistance = 0.2
        self.position = [ 0.0 ] * len(self.dims)
        self.stds = [ 0.0 ] * len(self.dims)
        # The threshold is a number beyond which we can't reliably
        # seek a goal, so the behavior has to change from seeking a
        # goal to determining the orientation.
        self.stdthres = stdthres
        # We maintain the following so we can do derivative-proportional
        # damping of the motion.  (It's a PD controller.)
        self.twist = Twist()
        self.debug = debug
        self.dbgdir = dbgdir # Where to put data files.
        self.count = 0
        self.targetFn = targetFn
        self.signalFn = signalFn
        self.maxTurn = maxTurn
        self.maxSpeed = maxSpeed
        self.forceList = force.ForceList(maxSpeed, maxTurn,
                                         linDamp, angDamp,
                                         debug=self.debug)
        # The fudge factor is for controlling the position
        # extrapolation.  The processing of each guess takes long
        # enough that there is an appreciable lag between the most
        # recent measurement and the current time.  In order to get
        # the force to work out right, we have to extrapolate the
        # current position from the last measurement and the speed
        # we're moving at the time.  The fudge factor is a part of
        # that calculation, and you may need to adjust it for
        # different speeds and floor surfaces.
        self.fudge = fudge

        # Use this to signal that the updateLocation should stop
        # working for a moment.  (i.e. we have some motion emergency
        # to deal with, like a bump)
        self.isbusy = False

        # These are for tracking the position
        self.debugX = 0.0
        self.debugY = 0.0
        self.debugTh = 0.0
        rospy.Subscriber("position", Position, self.debug_track)

        # Use this to keep updateLocation from being called too often.
        self.needUpdate = 0


    def setFudge(self, req):
        self.fudge = req.fudge

    def updateLocation(self, req):
        """
        This method is used to move the robot.  It gets the location,
        passes it to the force calculation and moves the robot
        accordingly.
        """
        self.needUpdate = self.needUpdate % 8.0
        self.needUpdate += 1
        if self.

        if isinstance(req, Belief):
            # Update estimate of current position
            # We have a position estimate, but it's from data measured
            # at the following fix time.
            fixTime = req.source_stamp.to_sec()
            currentTime = rospy.get_time()
            dT = currentTime - fixTime

            # Estimate movement since fix.
            dR = self.twist.linear.x * dT * self.fudge * 1.08
            dTh = self.twist.angular.z * dT * self.fudge * 0.95

            self.position = (req.means[0] + dR * math.cos(req.means[2] + dTh * 0.5),
                             req.means[1] + dR * math.sin(req.means[2] + dTh * 0.5),
                             req.means[2] + dTh)
            self.stds = req.stds

            if self.debug:
                print "*\nTime: %.3f - %.3f = %.3f" % (currentTime, fixTime, dT)
                print "Guessed position: (%5.3f,%5.3f,%5.3f)" % req.means
                print "Last position data: %s" % (req.source_data,)
                print "stds: (%5.3f,%5.3f,%5.3f)" % req.stds
                print "est:  (%5.3f,%5.3f,%5.3f) fudge=%5.3f" % (self.position[0], self.position[1], self.position[2], self.fudge)
                print "speeds x=%5.3f, z =%5.3f" % (self.twist.linear.x, self.twist.angular.z)
                print "from dr=%5.3f, dth=%5.3f" % (dR, dTh)

        elif isinstance(req, Position):
            self.position = ( req.x, req.y, req.theta )
            self.stds = ( 0.0, 0.0, 0.0 )

        if self.isbusy:
            return

        # If we're there, pop the goal stack and execute the victory
        # function.
        if self.achievedGoal():
            self.popGoal()
            self.targetFn(self.position)

        # If we don't know where we are, sit and spin in the hope we'll see
        # an AR tag or some other navigation aid will come to the rescue.
        if ((self.stds[0] + self.stds[1])/2.0) > self.stdthres:
            self.twist.linear.x = 0.4 * self.maxSpeed
            self.twist.angular.z = 0.6 * self.maxTurn
        # If there's still a goal on the stack, sum the forces.
        elif self.gs.top():
            self.twist = self.forceList.sumForces(self.position, self.twist)
        # If there's no goal on the stack and we know where we are, then stop.
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        if self.debug:
            self.count += 1
            filename = self.dbgdir + ("/%05dpts.txt" % (self.count,))
            f = open(filename, "w")
            f.write(str(self.count) + "\n")
            f.write(str(self.twist) + "\n")
            f.write("PT, %8.4f, %8.4f, %8.4f\n" % (self.debugX, self.debugY, self.debugTh))
            f.write(str(req) + "\n")
            f.close()

        # Change the motion commands
        self.pub.publish(self.twist)

    def debug_track(self, req):
        self.debugX = req.x
        self.debugY = req.y
        self.debugTh = req.theta

    # Checks to see if we're more or less at the goal on the top of the stack.
    # TODO: This ignores the periodic dimensions.
    def achievedGoal(self):
        if self.gs.top():
            dist = self.distance(self.position, self.gs.top().coords)
            return dist < self.zeroDistance
        else:
            return False # No goals in stack.

    # Returns the root of the sum of the squares for all non-periodic dimensions.
    def distance(self, pos_start, pos_end):
        ds = map(lambda st, en, pd: 0 if pd else en - st,
                pos_start, pos_end, self.dims)

        # Is this more python-ish?  Do we care?
#        ds = [0 if pd else en - st for st, en, pd in \
#                  zip(pos_start, pos_end, self.dims)]

        return sum([ d**2 for d in ds ])**0.5

    # Add a goal to get to first.
    def pushGoal(self, newGoal):

        if self.debug:
            print "pushing goal:", newGoal.str()
            print self.gs.str()
            print self.forceList.str()
        # If there is a current top goal, remove the force corresponding to it
        # from the force list.
        if self.gs.top():
            self.forceList.removeForce(self.gs.top().goalPtr)

        # Push the goal onto the stack.
        index = self.gs.push(newGoal)

        # Create a force for this goal.
        goalForce = force.ForceVector(newGoal.coords, debug=self.debug)

        # Add the new force to the list of forces.
        forceIndex = self.forceList.addForce(goalForce)

        # Record its position on the force list in the goal object.
        self.gs.top().setPtr(forceIndex)

        if self.debug:
            print "result: index:%d, findex:%d" % (index, forceIndex)
            print self.gs.str()
            print self.forceList.str()

        return index

    # Remove a goal.  Either we've gotten there or given up.
    def popGoal(self, index = 0):
        if self.gs.top():

            if self.debug:
                print "popping goal %d" % index
                print self.gs.str()
                print self.forceList.str()
                forceIndex = 0

            # Pop the goal from the stack
            oldGoal = self.gs.pop(index)

            if self.debug:
                print "popped:%s (%d)" % (oldGoal.str(), oldGoal.goalPtr)

            # Remove the corresponding force from the force list.
            self.forceList.removeForce(oldGoal.goalPtr)
        else:
            return False

        # If there's still a goal on the stack...
        if self.gs.top():
            # Create a force to get us to this new goal.
            goalForce = force.ForceVector(self.gs.top().coords)

            # Add the new force to the force list.
            forceIndex = self.forceList.addForce(goalForce)

            # Record its position on the force list in the goal object.
            self.gs.top().setPtr(forceIndex)

        if self.debug:
            print "result: index:%d, findex:%d" % (index, forceIndex)
            print self.gs.str()
            print self.forceList.str()

        return oldGoal

    def updateGoal(self, index, newGoal):
        """
        Change a goal -- and the resulting force if it has one.
        (Which should imply that it's the current top goal on the
        stack.)
        """

        newGoal = self.gs.update(index, newGoal)
        if newGoal.goalPtr:
            return self.forceList.updateForce(newGoal.goalPtr, newGoal.coords)
        else:
            return False

    def perturbGoal(self, req):
        """
        We have hit some object and need to move back.  Find the
        current position, move back and to the side a little bit, and
        try again.
        """
        if not self.isbusy:
            if req.bumpLeft or req.bumpRight:
                # Make sure we won't be interrupted.
                self.isbusy = True

                self.addForce("F", self.position)

                self.signalFn(11)
                # Create, and then publish a directive to back up.
                rev = Twist()
                rev.linear.x = -self.maxSpeed
                direction = -1.0 if req.bumpLeft else 1.0
                rev.angular.z = direction * self.maxTurn
                self.pub.publish(rev)

                rospy.sleep(2.0)
                rev.linear.x = self.maxSpeed
                rev.angular.z = direction * self.maxTurn
                self.pub.publish(rev)
                rospy.sleep(2.7)
                self.pub.publish(Twist())
                rospy.sleep(0.1)

                # Sing about it.
                self.signalFn(15)
                print "*******************************************OUCH****"

                # Ok to be interrupted now.
                self.isbusy = False

    def dispatchGoalRequest(self, req):
        if req.operation == 1:
            out = self.pushGoal(goal.Goal(req.position))
        elif req.operation == 2:
            out = self.popGoal(index = req.index)
        elif req.operation == 3:
            out = self.updateGoal(req.index, goal.Goal(req.position))
        else:
            out = 0
        return GoalResponse(out)

    def addForce(self, forceType, nodePos):
        if self.debug:
            self.count += 1
            filename = self.dbgdir + "/forces.txt"
            f = open(filename, "a")
            f.write(("%.2f:%s: " % (rospy.Time.now().to_sec(),
                                    forceType)) + \
                         ("(%.4f,%.4f,%.4f)\n" % nodePos))
            f.close()

        if forceType == "V":
            newForce = force.ForceVector(nodePos)
            return self.forceList.addForce(newForce)
        elif forceType == "F":
            newForce = force.ForceField(nodePos)
            return self.forceList.addForce(newForce)
        else:
            return 0

    def removeForce(self, index):
        return self.forceList.removeForce(index)

    def updateForce(self, index, newPos):
        return self.forceList.updateForce(index, newPos)

    def dispatchForceRequest(self, req):
        if req.operation == 1:
            out = self.addForce(req.forceType, req.nodePosition)
        elif req.operation == 2:
            out = self.removeForce(req.index)
        elif req.operation == 3:
            out = self.updateForce(req.index, req.nodePosition)
        else:
            out = 0
        return ForceResponse(out)


if __name__ == '__main__':

# Testing

    class FakePublisher(object):
        def __init__(self):
            self.tw = Twist()

        def publish(self, tw):
            self.tw = tw

        def str(self):
            return str(self.tw)

        def nop(self, x):
            return

    import unittest
    import math
    class TestForces(unittest.TestCase):
        def setUp(self):
            self.pub = FakePublisher()
            self.stg = StepToGoal((0.0,0.0,2*math.pi), self.pub,
                                  0.5, 1.0, 1.0, 2.0, targetFn=pub.nop)

            self.ffa = ForceField((0.25,0.0))
            self.ffb = ForceField((0.25,0.3))
            self.ffc = ForceField((-0.25,-0.3))
            self.ffd = ForceField((-0.25,0.0))

            self.fva = ForceVector((1.0, 1.0, 22.0/7.0))
            self.fvb = ForceVector((-1.0, -1.0, 0.8))

            self.fl.addForce(self.ffa)
            self.fl.addForce(self.ffb)
            self.fl.addForce(self.ffc)
            self.fl.addForce(self.fva)
