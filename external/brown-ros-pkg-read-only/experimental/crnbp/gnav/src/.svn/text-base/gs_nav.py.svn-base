#!/usr/bin/env python
#
# Takes a collection of points representing a robot's belief about its own
# position and publishes a Twist object designed to get us there relatively
# soon.
#
import roslib ; roslib.load_manifest('gnav')
import rospy
import numpy as np
from perfesser.msg import Belief, Pt
from position_tracker.msg import Position
from geometry_msgs.msg import Twist
from perfesser_guesser import Guesser
import math
import force
import goal

class GuessNav(object):
    def __init__(self,
                 dims = (0.0,0.0,2*math.pi),
                 publisher = False,
                 maxSpeed = 0.15,
                 maxTurn = 0.3,
                 dampSpeed = 1.0,
                 dampTurn = 1.0,
                 kSpeed = 1.0,
                 kTurn = 1.0,
                 stdThres=100.0,
                 zeroDistance=0.3,
                 targetFn=False,
                 signalFn=False,
                 debug=False, dbgdir="~/"):
        self.maxSpeed = maxSpeed
        self.maxTurn = maxTurn
        self.dampSpeed = dampSpeed
        self.dampTurn = dampTurn
        self.kSpeed = kSpeed
        self.kTurn = kTurn

        self.dims = dims
        self.pub = publisher

        self.targetFn = targetFn
        self.signalFn = signalFn

        self.debug = debug
        self.dbgdir = dbgdir
        self.count = 0
        # These are for tracking the position
        self.debugX = 0.0
        self.debugY = 0.0
        self.debugTh = 0.0
        if self.debug:
            rospy.Subscriber("position", Position, self.debug_track)

        self.fl = force.ForceList()
        self.gs = goal.GoalStack()

        self.zeroDistance = zeroDistance
        self.position = [ 0.0 ] * len(self.dims)
        self.realPosition = [ 0.0 ] * len(self.dims)
        self.stds = [ 0.0 ] * len(self.dims)

        self.curstds = 1000.0

        # The threshold is a number beyond which we can't reliably
        # seek a goal, so the behavior has to change from seeking a
        # goal to determining the orientation.
        self.stdThres = stdThres

        # We maintain the following so we can do derivative-proportional
        # damping of the motion.  (It's a PD controller.)
        self.twist = Twist()

        # Use this to signal that the doMove should stop
        # working for a moment.  (i.e. we have some motion emergency
        # to deal with, like a bump)
        self.isbusy = False

        # These two time stamps are used to control the bump behavior.
        self.bumpTimeOne = 0.0
        self.bumpTimeTwo = 0.0
        # Use these to remember where the current bump was.
        self.bumpLeft = False
        self.bumpRight = False

        # A flag to indicate whether we know where we are to within the
        # stdThres.  We use the flag to do stuff that has to happen *when*
        # we get lost, as opposed to stuff that has to happen *while* we
        # are lost, which is indicated with self.lost().
        self.amLost = False

        # This histogram is used to turn a force into a speed and
        # direction.  We try to keep the same histogram from one
        # invocation of set_move to the next in order to minimize the
        # adjustments to the speed and direction of the robot.
        self.guesser = Guesser("forces", (0.0, 2*math.pi), "force", (3,3))
        self.guesser.makeHist()
        self.guesser.hist.all()
        #print ">>>>>", self.guesser.hist.bins

        


    def distance(self, pos_start, pos_end):
        """
        Pythagorean distance between two points.  Ignores periodic
        dimensions.
        """
        ds = map(lambda st, en, pd: 0 if pd else en - st,
                pos_start, pos_end, self.dims)

        return sum([ d**2 for d in ds ])**0.5

    def achievedGoal(self):
        """
        Checks to see if we're more or less at the goal on the top of
        the stack.
        """
        if self.gs.top():
            dist = self.distance(self.position, self.gs.top().coords)
            return dist < self.zeroDistance
        else:
            return False # No goals in stack.

    def pushGoal(self, newGoal):
        """
        Pushes a goal onto the top of the goal stack.
        """

        # If there is a current top goal, remove the force corresponding to it
        # from the force list.
        if self.gs.top():
            self.fl.removeForce(self.gs.top().goalPtr)

        # Push the goal onto the stack.
        index = self.gs.push(newGoal)

        # Create a force for this goal.
        goalForce = force.ForceAttract(newGoal.coords, debug=self.debug)

        # Add the new force to the list of forces.
        forceIndex = self.fl.addForce(goalForce)

        # Record its position on the force list in the goal object.
        self.gs.top().setPtr(forceIndex)

        return index

    def popGoal(self, index = 0):
        """
        Remove a goal.  Either we've gotten there or given up.
        """
        if self.gs.top():

            # Pop the goal from the stack
            oldGoal = self.gs.pop(index)

            # Remove the corresponding force from the force list.
            self.fl.removeForce(oldGoal.goalPtr)
        else:
            return False

        # If there's still a goal on the stack...
        if self.gs.top():
            # Create a force to get us to this new goal.
            goalForce = force.ForceAttract(self.gs.top().coords, debug=self.debug)
            # Add the new force to the force list.
            forceIndex = self.fl.addForce(goalForce)

            # Record its position on the force list in the goal object.
            self.gs.top().setPtr(forceIndex)

        return oldGoal

    def updateGoal(self, index, newGoal):
        """
        Change a goal -- and the resulting force if it has one.
        (Which should imply that it's the current top goal on the
        stack.)
        """
        newGoal = self.gs.update(index, newGoal)
        if newGoal.goalPtr:
            return self.fl.updateForce(newGoal.goalPtr, newGoal.coords)
        else:
            return False

    def doLostMove(self, belief):
        """
        We don't know where we are.  Executes a strategy to find a
        landmark tag.  Basically this just means going somewhere with
        an arc, to maximize the chance of seeing a landmark.
        """
        if not self.amLost:
            self.signalFn(13)
            self.twist.linear.x = 0.8 * self.maxSpeed
            self.twist.angular.z = 0.2 * self.maxTurn
            self.twist.angular.z *= 1.0 if np.random.random() > 0.5 else -1.0
            self.amLost = True
        return

    def doLostBump(self, req):
        """
        We're lost, meaning that our position belief isn't very
        useful.  (Variances > self.stdThres, see self.lost().)
        There's no point in recording the obstacle position in this
        case.  All we need to do is pick another direction to go in,
        rotate to it, and go.
        """
        tnow = rospy.Time.now().to_sec()
        #print "entering doLostBump:", self.bumpTimeOne, self.bumpTimeTwo, tnow

        if self.bumpTimeTwo > 0.0:
            if tnow > self.bumpTimeTwo:
                self.bumpTimeTwo = 0.0
                self.bumpLeft = False
                self.bumpRight = False

                if self.lost():
                    # Resume search, but try to edge away from the obstacle.
                    out = self.twist
                    out.angular.z *= -1.0 if self.bumpLeft else 1.0

                else:
                    # Stop and await further instruction.
                    out = Twist()
            else:
                return

        elif self.bumpTimeOne > 0.0:
            if tnow > self.bumpTimeOne:
                self.bumpTimeTwo = self.bumpTimeOne + 3.0 * np.random.random()
                self.bumpTimeOne = 0.0

                # Rotate some random amount
                out = Twist()
                out.linear.x = 0.0
                out.angular.z = (-1.0 if self.bumpLeft else 1.0) * self.maxTurn

            else:
                return

        else:
            self.bumpTimeOne = rospy.Time.now().to_sec() + 2.0
            self.bumpLeft = req.bumpLeft
            self.bumpRight = req.bumpRight

            out = Twist()
            out.linear.x = -self.maxSpeed
            direction = -1.0 if req.bumpLeft else 1.0
            out.angular.z = direction * self.maxTurn

        self.pub.publish(out)

        return

    def doBump(self, req):
        """
        We're not lost, which means we have a goal in mind.  So find the
        current position, move back and to the side a little bit, and
        try again.  Also record the position of the obstacle and maybe
        we can avoid bumping it again.
        """
        tnow = rospy.Time.now().to_sec()
        #print "entering doBump:", self.bumpTimeOne, self.bumpTimeTwo, tnow

        if self.bumpTimeTwo > 0.0:
            if tnow > self.bumpTimeTwo:
                # Stop the robot, clear the decks.
                self.bumpTimeTwo = 0.0
                self.bumpLeft = False
                self.bumpRight = False

                out = Twist()

                # Sing about it.
                if self.signalFn:
                    self.signalFn(15)

            else:
                return

        elif self.bumpTimeOne > 0.0:
            if tnow > self.bumpTimeOne:
                self.bumpTimeTwo = self.bumpTimeOne + 2.7
                self.bumpTimeOne = 0.0

                # Go straight back for a little bit.
                out = Twist()
                out.linear.x = self.maxSpeed
                out.angular.z = -self.maxTurn if self.bumpLeft else self.maxTurn

            else:
                return

        else:
            # Infer the position of the obstacle from the position of the
            # robot.
            angleCorrection = self.position[2] + \
                ( 0.7 if req.bumpLeft else 0.0) + (-0.7 if req.bumpRight else 0.0)
            self.addForce("R", (self.position[0] + .3 * math.cos(angleCorrection),
                                self.position[1] + .3 * math.sin(angleCorrection),
                                self.position[2]), duration=60.0)

            #print "obstacle at: %.3f,%.3f,%.3f" % \
            #    (self.position[0] + .3 * math.cos(angleCorrection),
            #     self.position[1] + .3 * math.sin(angleCorrection),
            #     self.position[2])

            # Sing about it
            if self.signalFn:
                self.signalFn(5)

        # Create, and then publish a directive to back up.
            self.bumpTimeOne = rospy.Time.now().to_sec() + 2.0
            self.bumpLeft = req.bumpLeft
            self.bumpRight = req.bumpRight

            out = Twist()
            out.linear.x = -self.maxSpeed
            out.angular.z = - self.maxTurn if req.bumpLeft else self.maxTurn

        self.pub.publish(out)

        return

    def perturbGoal(self, req):
        """
        We have hit some object and need to do something about it.
        The action we take depends on whether we're lost or not.  If
        we're lost, just rotate some random amount and go.
        """
        if not self.isbusy:
            if req.bumpLeft or req.bumpRight or \
                    self.bumpTimeOne > 0.0 or self.bumpTimeTwo > 0.0:
                # Make sure we won't be interrupted.
                self.isbusy = True

                print "*******************************************OUCH****"

                if self.lost():
                    self.doLostBump(req)
                else:
                    self.doBump(req)

                # Ok to be interrupted now.
                self.isbusy = False

    def addForce(self, forceType, nodePos, duration=3600.0):
        if forceType == "A":
            newForce = force.ForceAttract(nodePos, duration=duration)
            return self.fl.addForce(newForce)
        elif forceType == "R":
            newForce = force.ForceRepel(nodePos, duration=duration)
            return self.fl.addForce(newForce)
        else:
            return 0

    def purgeForces(self):
        return self.fl.purgeForces()

    def removeForce(self, index):
        return self.fl.removeForce(index)

    def updateForce(self, index, newPos):
        return self.fl.updateForce(index, newPos)

    def lost(self):
        """
        Returns True if the uncertainty in the current position is
        higher than the threshold set at initialization.
        """
        return (((self.stds[0] + self.stds[1])/2.0) > self.stdThres)

    def setModelPosition(self, fixTime):
        """
        Given a twist and a delta T, calculate how much the robot has moved.
        Store the result so it can be added quickly with fixPosition().
        """
        currentTime = rospy.Time.now().to_sec()
        self.dT = currentTime - fixTime

        self.modeldR = self.twist.linear.x * self.dT * 1.08
        self.modeldTh = self.twist.angular.z * self.dT * 0.95
        return

    def modelPosition(self, point):
        """
        Apply the correction calculated in setModelPosition().
        """
        return (point[0] + self.modeldR * math.cos(point[2] + 0.5 * self.modeldTh),
                point[1] + self.modeldR * math.sin(point[2] + 0.5 * self.modeldTh),
                point[2] + 0.7 * self.modeldTh)

    def doForceMove(self, belief):
        """
        Runs through the points in the input belief and calculates the
        'force' on each one.  Uses the result to come up with a magnitude
        and direction relative to the robot's position, then translates
        this into a Twist message, which is published.
        """
        # Update all the points in the belief with the model position
        # adjustment.  Then calculate the 'force' on each point.
        forceList = []
        for pt in belief.points:
            modelPt = self.modelPosition(pt.point)

            fX, fY = self.fl.sumForces(modelPt)

            # Convert to polar coordinates.
            fM = math.hypot(fX, fY)
            fD = math.atan2(fY, fX)

            # Convert to robot frame
            fDR = fD - modelPt[2]
            if math.fabs(fDR) > math.pi:
                fDR -= math.copysign(2 * math.pi, fDR)

            forceList.append(Pt(point = (fM, fDR)))

        # We have a long collection of forces, one for each point in the
        # belief.  We feed them to the guesser in order to get a decent
        # estimate of the mode and to use the guesser's binning functions
        # in making that estimate.
        self.guesser.newPoints(forceList)
        means = self.guesser.means()
        # This is the mean force direction, more or less.
        mfDirs = means[1]


        binmeans,binwidths = self.guesser.hist.binmeans()
        #print "self.position: ", self.position
        #print "%s, %s, %.3f" % (("(%.2f,%.2f)" % tuple(binmeans)),
        #                        ("(%.2f,%.2f)" % tuple(binwidths)),
        #                        mfDirs)
        #print self.guesser.hist.bins

        # How many bins should we use on the force angle when calculating a
        # mode?  The closer we are to going in the right direction, the tighter
        # the bins should get.  But there aren't enough points for too many
        # bins, so limit the number.
        nAngBins = min(11, 5 + 2 * (int(1.57 / math.fabs(mfDirs))))
        #print "::::",nAngBins, self.guesser.hist.numBins[1]

        # Decide whether or not to regenerate the bin boundaries.
        if (nAngBins != self.guesser.hist.numBins[1]) or \
                (math.fabs(binmeans[1] - mfDirs) > binwidths[1]/2.0):
            self.guesser.hist.rebin(self.guesser.pointArray, (3,nAngBins))
            #print "regenerating (angbins=%d)" % nAngBins
        else:
            self.guesser.hist.populate(self.guesser.pointArray)

        #print self.guesser.hist.str()

        # Choose the mode from the histogram and generate a decent force
        # value from its boundaries.
        fM,fDR = self.guesser.hist.mode()

        # Translate this into a Twist object, applying appropriate damping
        # and maxima.
        fDR = fDR % (2 * math.pi)
        fDR = fDR - (2 * math.pi) if fDR > math.pi else fDR

        print "fDR = ", fDR
        # Calculate some provisional speeds, later to be subject to damping.
        if math.fabs(fDR) > 0.2:
            # If we're pointing the wrong way, just rotate (linear = 0).
            provX = 0.0
        else:
            provX = min(self.maxSpeed, self.kSpeed * fM)

        provZ = math.fabs(self.kTurn * fDR)
        provZ = min(self.maxTurn, provZ)
        provZ = math.copysign(provZ, fDR)

        print "provX, provZ", provX, provZ

        # If the last location estimate had a lower variance than this one, 
        # ignore the new one.
        if provX > 0.0 and \
                (self.curstds < 0.5 * (belief.stds[0] + belief.stds[1])):
            print "out of here..."
            return

        self.curstds = 0.5 * (belief.stds[0] + belief.stds[1])
        
        # Apply damping and put it into the twist message.
        if provX:
            self.twist.linear.x += self.dampSpeed * (provX - self.twist.linear.x)
        else:
            self.twist.linear.x = provX

        self.twist.angular.z += self.dampTurn * (provZ - self.twist.angular.z)

        return

    def doMove(self, belief):
        assert isinstance(belief, Belief)

        # The process of calculating all the beliefs takes time, and so the
        # belief in hand is actually a tiny bit old.  We use a model of the
        # robot motion to advance each point of the belief to the more or less
        # current time.  Calling setModelPosition() chooses an incremental
        # advance which will then be applied to all the points.
        self.setModelPosition(belief.source_stamp.to_sec())
        # This is just an estimated position used in debugging.
        self.position = self.modelPosition(belief.means)
        self.stds = belief.stds

        # If we're there, pop the goal stack and execute the victory
        # function.
        if self.achievedGoal():
            self.popGoal()
            if self.targetFn:
                self.targetFn(self.position)

        # This is just in case we're dealing with a bump or something.
        if self.isbusy or self.bumpTimeOne > 0.0 or self.bumpTimeTwo > 0.0:
            # print "nothing doing"
            return

        # Get rid of any forces that should have been forgotten.
        self.fl.purgeForces()

        #print "doMove", self.lost(), self.gs.top(), self.amLost

        if self.lost():
            # We don't know where we are.  Sit and spin and hope we see a
            # tag.
            self.doLostMove(belief)

        elif self.gs.top():
            self.amLost = False
            # If there's still a goal on the stack, calculate forces and go.
            self.doForceMove(belief)

        else:
            self.amLost = False
            # There is no goal on the stack and we know where we are.  Stop.
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        if self.debug:
            self.count += 1
            filename = self.dbgdir + ("/%05dpts.txt" % (self.count,))
            f = open(filename, "w")
            f.write(str(self.count) + "\n")
            f.write(str(self.twist) + "\n")
            f.write("PT, %8.4f, %8.4f, %8.4f\n" % (self.debugX, self.debugY, self.debugTh))
            f.write(str(belief) + "\n")
            f.close()

        # Publish whatever Twist message we've produced.
        if self.bumpTimeOne + self.bumpTimeTwo > 0.0:
            return

        self.pub.publish(self.twist)


    def getRealRobotPos(self, tps):
        """
        A debugging function, meant to get a position from some external
        process, to be incorporated into a debugging message.
        """
        for tp in tps.tag_positions:
            if tp.id == 2:
                self.debugX = tp.x
                self.debugY = (36+39) - tp.y
                self.debugTh = math.pi - tp.theta
                #print "planar report: (%.3f,%.3f,%.3f)" % (self.debugX, self.debugY, self.debugTh)


    def debug_track(self, req):
        self.debugX = req.x
        self.debugY = req.y
        self.debugTh = req.theta

