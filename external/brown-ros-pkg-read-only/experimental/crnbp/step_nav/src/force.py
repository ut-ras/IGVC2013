#!/usr/bin/env python
# This is a collection of classes designed to provide a way to
# simulate a potential field that will move each of the robots.  There
# is a Force base class, meant to take a robot's pose (position and
# orientation) and return a "force" on it (expressed as a correction
# to the current speed).
#
# There are several variations of the base Force available.  Each
# robot has a ForceList object to keep a list of Forces together and
# at any one time, the force that acts on a particular robot will be
# the sum of all the Force objects in the list.

# TODO: This is strictly 2-d now.  Should be made more general, using
# Twist objects to contain the input and output speeds, as well as the
# limits and damping constants.  Q: can we iterate over a Twist?
# The Force functions can remain general, but in addition to the
# engineering constants for damping and limits, the ForceList object
# should contain an array that indicates what kinds of output forces
# are actually useful.  That is, an irobot create can't use the
# linear.y or linear.z axes and I gather that the AR drones can't use
# angular.x or angular.y.
# TODO: Could use a random-walk force, for perturbing the system's
# equilibrium, to allow a kind of "annealing" solution to be worked
# out in non-model space.
import roslib ; roslib.load_manifest('step_nav')
import math
import numpy as np
from geometry_msgs.msg import Twist

# This is the abstract base class inherited by all the force variants.
# The idea for each one is that you have a force exerted on a robot as
# a function of its position.  The force is based on a point in space
# (the "node"), which might be an attractor or a repeller or whatever.
# Some dimensions of the node might be ignored, some might mean
# something unusual, it doesn't matter.  All that matters is that the
# result is expressed as a dV to be applied to the V currently in
# effect.  Subclasses of this class are the objects contained in the
# ForceList, below.
class Force(object):
    def __init__(self, forceFunction, ftype=""):
        self.forceFunction = forceFunction
        self.ftype = ftype
        self.nodePos = 0

    # The heart of each Force object is a force() method to return a
    # dV for an input position in space.
    def force(self, pos):
        return self.forceFunction(pos)

    # Each force has a "node" on which it's based.  The meaning of the
    # node's position is up to the implementation, depending on
    # whether the force is an attractors, a repeller, or something else.
    def setNodePos(self, newNodePos):
        self.nodePos = newNodePos

    # Provides a string representation of the force function pos.
    def str(self):
        slist = [" {0:.3f},".format(p) for p in self.nodePos]
        s  = reduce(lambda x,y: x+y, slist)
        return "{0}:({1})".format(self.ftype, s[1:-1])

    # These functions are because the rest of the world speaks in
    # Twist() objects, but you can't iterate over them, so they're a
    # pain to deal with when making parallel calculations.  In here,
    # we use a different way to represent poses, a tuple (or list) of
    # coordinates, accompanied by a list of the dimensions, where 0
    # indicates a non-periodic dimension and non-zero dimensions
    # contain their period.  So, (x,y,theta) has a dim array of
    # (0,0,2pi).
    def unpackTwist(self, twist):
        return (twist.linear.x, twist.linear.y, twist.angular.z)

    def repackTwist(self, pos):
        tw = Twist()
        tw.linear.x = pos[0]
        tw.angular.z = pos[2]
        return tw

# This Force implementation produces a 1/r**4 repeller from the node
# position.
class ForceField(Force):
    def __init__(self, node, debug=False):
        Force.__init__(self, self.forceFromField, ftype="FF")
        self.nodePos = node
        self.debug = debug

    def forceFromField(self, pos):
        """
        Calculates a force vector based on a position in some field.
        We're walking down a gradient here and do not necessarily have
        a goal, except to minimize our 'height' in the force field.
        This is similar to forceFromVector except there is no goal.
        """
        X = pos[0] ; Y = pos[1] ; Theta = pos[2]

        pVX = X - self.nodePos[0]
        pVY = Y - self.nodePos[1]

        forceFactor = 0.1
        fVM = forceFactor/(pVX**2 + pVY**2)**2
        fVD = math.atan2(pVY, pVX)

        fVN = fVM * math.cos(fVD)
        fVT = fVM * math.sin(fVD)

        # Translate back into reference frame.
        #fVNR = fVX * math.cos(Theta) + fVY * math.sin(Theta)
        #fVTR = -fVX * math.sin(Theta) + fVY * math.cos(Theta)

#        if self.debug:
#            print "pX=%5.3f,pY=%5.3f,fM=%5.3f,fD=%5.3f,fX=%5.3f,fY=%5.3f,fNM=%5.3f,fTM=%5.3f" % (pVX, pVY, fVM, fVD, fVX, fVY, fVN, fVT)

        return (fVN, fVT)

# Produces an attractor to the node position, currently implemented as
# a force proportional to the distance between the robot and the node.
#
class ForceVector(Force):
    def __init__(self, nodePos, debug=False):
        Force.__init__(self, self.forceFromVector, ftype="FV")
        self.nodePos = nodePos
        self.debug = debug

    def forceFromVector(self, pos):
        """
        Calculates two force vectors on the robot, a 'normal' force
        (positive = straight ahead) and a 'tangent' force (positive =
        counterclockwise from top -- right-hand-rule).  The
        calculations are done in the frame of the robot's current
        position, and then transformed to the reference frame in which
        the robot sits.
        """
        # Calculate differences
        X = self.nodePos[0] - pos[0] ; Y = self.nodePos[1] - pos[1]
        Theta = pos[2]

        if self.debug:
            print "Theta:", Theta
        Theta = Theta % (2 * math.pi)
        if Theta > math.pi:
            Theta -= 2 * math.pi
            if self.debug:
                print "adjusting theta: %6.3f" % (Theta,)

        forceFactor = 20.0
        # The tangent force is calculated from the robot's center, but
        # will be applied as a torque, so you can think of it as
        # acting on the robot's rim or something.  This is a bit of a
        # hack, but will be cared for with the engineering constants
        # used.
        # 'fV' = 'force vector', M = 'magnitude', D = 'direction',
        # N = 'normal', T = 'tangent'
        fVM = forceFactor * math.hypot(X, Y)
        fVD = math.atan2(Y, X)

        fVN = fVM * math.cos(Theta - fVD)
        fVT = -fVM * math.sin(Theta - fVD)

        # If we're close enough to be already there, just fix the rotation.
        if math.hypot(X, Y) < 0.1:
            fVN = 0.0
            fVT = -forceFactor * Theta

        if self.debug:
            print "Position -> X=%5.3f, Y=%5.3f, Th=%5.3f" % pos
            print "Headed for -> X=%5.3f, Y=%5.3f, Th=%5.3f" % self.nodePos
            print "dX=%5.3f,dY=%5.3f,dTh=%5.3f,fM=%5.3f,fD=%5.3f" % (X, Y, Theta - fVD, fVM, fVD)
#            print "fN=%5.3f,fT=%5.3f,fNR=%5.3f,fTR=%5.3f" % (fVN,
#            fVT, fVNR, fVTR)
            print "fN=%5.3f,fT=%5.3f" % (fVN, fVT)

        return (fVN, fVT)  ##################
#
# A map force is meant to provide repulsive forces from walls and other
# architectural obstacles.  The node position indicates the middle of
# the relevant portion of the map.  Presumably this will be the robot
# position, more or less, and will be swapped in and out as the robot
# moves around the big map.
#
# Note that this is *not* about map localization.  There is no
# feedback here on the position.  We take it as granted, and if there
# is some mismatch between the position we are given and the map we
# have, we issue a complaint.
class ForceMap(Force):
    def __init__(self, node, server, debug=False):
        Force.__init__(self, self.forceFromMap, ftype="FM")
        # Ideally this is in the middle of the map.
        self.nodePos = node
        # The server where we can refresh our little view of the map.
        self.server = server
        self.debug = debug
        self.nX = 10 ; self.nY = 10
        # This is the map itself.  Zeros are free space, ones are positively
        # identified walls.  Maybe we'll eventually implement hypotheses here,
        # too, but for now this is just two values: 0.0 and > 0.0.
        self.map = np.zeros((self.nX, self.nY))
        # The position of the southwest corner of the map.
        self.mapX = 0.0 ; self.mapY = 0.0
        # The distance between map points
        self.intX = 0.2 ; self.intY = 0.2

        self.map[4,5] = 1.0 ; self.map[3,6] = 1.0

    def updateMap(self, newMap):
        self.map = newMap


    def forceFromMap(self, pos):
        # Find where we are in the map.
        xi = int((pos[0] - self.mapX)/self.intX)
        yi = int((pos[1] - self.mapY)/self.intY)

        pts = []

        # Find nearest neighbors who have a different value than
        # is in xi, yi.
        for i in range(1,self.nX):
            xmn = max(0, xi - i) ; xmx = min(self.nX - 1, xi + i)
            ymn = max(0, yi - i) ; ymx = min(self.nY - 1, yi + i)

            y = ymn
            for x in range(xmn, xmx + 1):
                if self.map[x,y] > 0.5:
                    pts.append((x, y, self.map[x,y]))
            for y in range(ymn + 1, ymx + 1):
                if self.map[x,y] > 0.5:
                    pts.append((x, y, self.map[x,y]))
            for x in range(xmx - 1, xmn - 1, -1):
                if self.map[x,y] > 0.5:
                    pts.append((x, y, self.map[x,y]))
            for y in range(ymx - 1, ymn, -1):
                if self.map[x,y] > 0.5:
                    pts.append((x, y, self.map[x,y]))

            if pts:
                break

        if not pts:
            return (0.0, 0.0)

        print pts
        # Add up the contribution to the force from each cell we've found.
        forceX = 0.0 ; forceY = 0.0
        for pt in pts:
            forceX += xi - pt[0] ; forceY += yi - pt[1]

        print "at %d,%d feel %.1f,%.1f" % (xi,yi,forceX, forceY)

        # Is this a repulsion or attraction?
        # It's a repulsion if we're in an empty space and one or more of
        # the neighbors is filled space.  It's an attraction if we're in a
        # filled space (something's wrong) and there's an empty neighbor.
        forceX *= self.intX
        forceY *= self.intY
        d = math.hypot(forceX, forceY)
        forceX *= 1/d**4
        forceY *= 1/d**4

        print "at %d,%d feel %.1f,%.1f" % (xi,yi,forceX, forceY)

        # Transform back into the reference frame
        fVNR = forceX * math.cos(pos[2]) + forceY * math.sin(pos[2])
        fVTR = forceX * math.sin(pos[2]) + forceY * math.cos(pos[2])

        return (fVNR, fVTR)


# X X X X X X X X X X
# X X X X X X X X X X
# X X X X X X X X X X
# X X X X X X X X X X
# X X X X X X X X X X
# X X X O X X X X X X =(4,5)
# X X X X X X X X X X
# X X X X X X X X X X
# X X X X X X X X X X
# X X X X X X X X X X

#3,6 4,6 5,6 5,5 5,4 4,4 3,4 3,5



#
# This is a force meant to get the robot to move in a particular
# direction and speed.  Imagine a big planar attractor, whose origin
# is indicated by the node position.
class ForceAlign(Force):
    def __init__(self, node, debug=False):
        Force.__init__(self, self.forceFromAlignField, ftype="FA")
        self.nodePos = node
        self.debug = debug
        self.xOrigin, self.yOrigin = self.setOrigin(self.nodePos)

    def setNodePos(self, newPos):
        Force.setNodePos(self, newPos)
        self.xOrigin, self.yOrigin = self.setOrigin(self.nodePos)

    def setOrigin(self):
        return (self.nodePos[0] * math.cos(self.nodePos[2]),
                self.nodePos[0] * math.sin(self.nodePos[2]))


    def forceFromAlignField(self, pos):
        """Sort of a fudge, meant to get the robot to point in a particular direction and move with a particular velocity.  Used to steer multiple robots in sync.  The idea is just to park a giant attractor at (x, th) (a/k/a (nodePos[0],nodePos[2])) so the robots who are attracted to it move more or less in parallel."""

        # The force vector we want is equivalent to the Y dimension of the input
        # point in a rotated and translated coordinate system.
        pVX = (pos[0] - self.xOrigin) * math.cos(self.nodePos[2]) + \
            (pos[2] - self.yOrigin) * math.sin(self.nodePos[2])
        pVY = (pos[2] - self.yOrigin) * math.cos(self.nodePos[2]) - \
            (pos[0] - self.xOrigin) * math.sin(self.nodePos[2])

        forceFactor = 100.0
        fVX = 0
        fVY = forceFactor * pVY

        fVM = fVY
        fVD = nodePos[2] - (math.pi/2.0)
        fVN = fVM * math.cos(fVD)
        fVT = fVM * math.sin(fVD)

        # Transform back into the reference frame
        fVNR = fVN * math.cos(pos[2]) + fVT * math.sin(pos[2])
        fVTR = -fVN * math.sin(pos[2]) + fVT * math.cos(pos[2])

        return (fVNR, fVTR)

# This class is used to manage a collection of Force objects acting on
# some robot.  The Force objects are defined and added individually,
# but the force acting on any robot is returned with the sumForces()
# method.
#
# The class also provides some facility for managing the list, with
# addForce() and removeForce() methods.  You can also change a Force
# with the updateForce() method, though the only change it really
# supports is to modify the Force object's node position.  For more
# elaborate modifications, you should remove the Force object from the
# list and add a new one.
#
# <aside> For an easily controllable Force object, you could also
# define one that subscribes to a ROS Message and use that message to
# fidget with the Force object's actions.  I think it inadvisable to
# use Services unless you can be sure there is only one instantiation
# of the Force object.</aside>
#
# This class also contains robot-specific stuff here, including
# velocity limits and damping constants.  (Making it essentially a PD
# controller for the case of a force proportional to the distance from
# its node point.
#
# TODO: the maxima and the damping should be Twist objects.
class ForceList(object):
    def __init__(self, maxSpeed, maxTurn, speedDamp, turnDamp,
                 debug=False):
        self.flist = {}
        self.nlist = 0
        self.maxSpeed = maxSpeed
        self.maxTurn = maxTurn
        self.speedDamp = speedDamp
        self.turnDamp = turnDamp
        # Exposed for debugging
        self.normalForce = 0.0
        self.tangentForce = 0.0
        self.debug = debug

        self.normalList = []
        self.tangentList = []
        self.nSmooth = 5

    def getForce(self, index):
        return self.flist[index]

    def addForce(self, newForce):
        self.nlist += 1
        self.flist[self.nlist] = newForce
        return self.nlist

    def removeForce(self, index):
        if index in self.flist.keys():
            del self.flist[index] # This only deletes the dict entry
            return True
        else:
            return False

    def updateForce(self, index, newPos):
        if index in self.flist.keys():
            self.flist[index].setNodePos(newPos)
            return True
        else:
            return False

    # Prints a representation of the force list.
    def str(self):
        out = ""
        for k in self.flist.keys():
            out += "F%d: %s\n" % (k, self.flist[k].str())
        return out[0:-1] # Lose the last \n

    def sumForces(self, pos, twist):
        """
        Provides the sum of the forces in the list on position pos.
        The twist is provided on input so we can do
        derivative-proportional damping.  Output is an updated twist,
        suitable for publication.
        """
        linearSpeed = twist.linear.x
        angularSpeed = twist.angular.z
        # Use this to corral forces within engineering bounds.  Note
        # that it also drops the force to zero if it's smaller than a
        # fraction of the minimum force.  Assume mn and mx are positive.
        bound = lambda mn,mx,x: math.copysign(min(mx, max(mn, math.fabs(x))),x) if (math.fabs(x) > (mn * 0.75)) else 0.0

        if self.flist:
            corrList = [ f.force(pos) for f in self.flist.values() ]

            if self.debug:
                for corr in corrList:
                    print "normalF: %5.3f, tangentF: %5.3f" % corr

            self.normalForce = sum([ c[0] for c in corrList ])
            self.tangentForce = sum([ c[1] for c in corrList ])

            # Robot sensors tend to be on the front, so we discourage backward
            # travel and rotate faster to get out of it.
            if self.normalForce < 0:
                self.normalForce /= 50.0
                self.tangentForce *= 6.0

            # Turn the force into a speed correction.  This involves
            # using the damping terms.
            normalCorrection = self.normalForce * self.speedDamp
            tangentCorrection = self.tangentForce * self.turnDamp

            self.normalList.append(normalCorrection)
            self.tangentList.append(tangentCorrection)
            if len(self.normalList) > self.nSmooth:
                self.normalList.pop(0)
                self.tangentList.pop(0)
            normalCorrection = sum(self.normalList)/len(self.normalList)
            tangentCorrection = sum(self.tangentList)/len(self.tangentList)

        else:
            normalCorrection = 0
            tangentCorrection = 0
            self.normalList = [] ; self.tangentList = []

#        outLinearSpeed = linearSpeed + normalCorrection
#        outAngularSpeed = angularSpeed + tangentCorrection
        outLinearSpeed = normalCorrection
        outAngularSpeed = tangentCorrection

        if self.debug:
            print "FL: norm=%5.3f, tang=%5.3f, fc=%5.3f,tc=%5.3f,x=%5.3f,z=%5.3f" % (self.normalForce, self.tangentForce, normalCorrection, tangentCorrection, outLinearSpeed, outAngularSpeed)

        # Apply the limiting terms and send it out.
        tw = Twist()
        tw.linear.x = bound(self.maxSpeed/25.0, self.maxSpeed, outLinearSpeed)
        tw.angular.z = bound(self.maxTurn/25.0, self.maxTurn, outAngularSpeed)

        # Complete hack, for debugging purposes only.  If used with
        # model_create, you'll see the force vectors drawn onto the
        # graphic.
        tw.linear.y = normalCorrection
        tw.angular.y = tangentCorrection

        return tw

if __name__ == '__main__':
# Testing

    import unittest
    class TestForces(unittest.TestCase):
        def setUp(self):
            self.fl = ForceList(0.5, 1.0, 1.0, 2.0)
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

        def testStackOperations(self):
            testString = "F1: FF:(0.250, 0.000)\nF2: FF:(0.250, 0.300)\nF3: FF:(-0.250, -0.300)\nF4: FV:(1.000, 1.000, 3.143)"
            self.assertEqual(self.fl.str(), testString)

            testString = "FF:(0.250, 0.300)"
            self.assertEqual(self.fl.getForce(2).str(),testString)

            self.assertEqual(self.fl.addForce(self.ffd),5)

            testString = "F1: FF:(0.250, 0.000)\nF2: FF:(0.250, 0.300)\nF3: FF:(-0.250, -0.300)\nF4: FV:(1.000, 1.000, 3.143)\nF5: FF:(-0.250, 0.000)"
            self.assertEqual(self.fl.str(), testString)

            self.assertTrue(self.fl.removeForce(1))
            self.assertFalse(self.fl.removeForce(32))

            testString = "F2: FF:(0.250, 0.300)\nF3: FF:(-0.250, -0.300)\nF4: FV:(1.000, 1.000, 3.143)\nF5: FF:(-0.250, 0.000)"
            self.assertEqual(self.fl.str(), testString)


            res = self.fl.updateForce(2,(1.25,1.0/3.0))
            self.assertTrue(res)

            testString = "F2: FF:(1.250, 0.333)\nF3: FF:(-0.250, -0.300)\nF4: FV:(1.000, 1.000, 3.143)\nF5: FF:(-0.250, 0.000)"
            self.assertEqual(self.fl.str(), testString)

        def testForceAddition(self):
            tw = Twist()
            outTw = self.fl.sumForces((0.0,0.0,0.0), tw)

            testString = "linear: \n  x: -0.312\n  y: -0.312\n  z: 0.0\nangular: \n  x: 0.0\n  y: 60.0\n  z: 1.0"
            self.assertEqual(str(outTw), testString)

            self.fl.addForce(self.fvb)

            tw = Twist()
            outTw = self.fl.sumForces((0.0,0.0,0.0), tw)

            testString = "linear: \n  x: -0.5\n  y: -0.512\n  z: 0.0\nangular: \n  x: 0.0\n  y: 1.06581410364e-14\n  z: 0.0"
            self.assertEqual(str(outTw), testString)


    suite = unittest.TestLoader().loadTestsFromTestCase(TestForces)
    unittest.TextTestRunner(verbosity=3).run(suite)


