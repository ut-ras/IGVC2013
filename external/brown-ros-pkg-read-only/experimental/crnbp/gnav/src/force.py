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
import roslib ; roslib.load_manifest('gnav')
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist

# This is the abstract base class inherited by all the force variants.
# The idea for each one is that you have a force exerted on a robot as
# a function of its position.  The force is based on a point in space
# (the "node"), which might be an attractor or a repeller or whatever.
# Some dimensions of the node might be ignored, some might mean
# something unusual, it doesn't matter.  All that matters is that the
# result is expressed as an X and Y magnitude in the world's reference
# frame.
#
# Subclasses of this class are the objects contained in the
# ForceList, below.
class Force(object):
    def __init__(self, forceFunction, ftype="", duration=3600.0):
        self.forceFunction = forceFunction
        self.ftype = ftype
        self.nodePos = 0
        self.expiry = rospy.Time.now().to_sec() + duration

    # The heart of each Force object is a force() method to return a
    # magnitude and direction for an input position in space.
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

class ForceRepel(Force):
    """
    Implements a 1/r**4 repeller from the node position.  The factor
    argument is used in the force calculation.
    """
    def __init__(self, node, factor=1.0, duration=3600.0, debug=False):
        Force.__init__(self, self.forceRepel, ftype="FR", duration=duration)
        self.nodePos = node
        self.factor = factor
        self.debug = debug

    def forceRepel(self, pos):
        """
        Calculates a force vector based on a position in some field.
        """
        pVX = pos[0] - self.nodePos[0]
        pVY = pos[1] - self.nodePos[1]

        fVM = self.factor/(pVX**2 + pVY**2)**2
        fVD = math.atan2(pVY, pVX)

        fVX = fVM * math.cos(fVD)
        fVY = fVM * math.sin(fVD)

        return (fVX, fVY)

class ForceAttract(Force):
    """
    Produces an attractor to the node position with force proportional
    to the distance.
    """
    def __init__(self, nodePos, factor=1.0, duration=3600.0, debug=False):
        Force.__init__(self, self.forceAttract, ftype="FA", duration=duration)
        self.factor = factor
        self.nodePos = nodePos
        self.debug = debug

        self.radius = 1000.0
    
    def forceAttract(self, pos):
        """
        An attractor proportional to the distance.  The direction of
        the force is meant to bring the robot on the quickest course
        into a shrinking circle around the node (target) point.  This
        is meant to ameliorate the problem that the closer you get,
        the more sensitive the robot's direction is to changes in the
        location estimate.  Not a perfect solution, but seems to
        behave slightly better than the simple attractor.
        """
        X = pos[0] - self.nodePos[0]
        Y = pos[1] - self.nodePos[1]

        dist = math.hypot(X, Y)
        self.radius = min(0.5, max(0.2, dist * 0.7))
        theta = math.atan2(Y,X)

        # Find the point on the circle of radius self.radius nearest pos.
        px = self.nodePos[0] + self.radius * math.cos(theta)
        py = self.nodePos[1] + self.radius * math.sin(theta)

        # Get the angle between the robot's pose and the line between it
        # and the target.  If we are more than pi/2 away, set the angle
        # to be +-p/2.
        diff = (pos[2] - theta + math.pi) % (2 * math.pi)
        diff = diff - (2 * math.pi) if diff > math.pi else diff
        if math.fabs(diff) > math.pi/2.0 :
            diff = math.copysign(math.pi/2.0, diff)

        dist2 = math.sin(diff) * self.radius/3.0
            
        qx = px + dist2 * math.sin(theta)
        qy = py - dist2 * math.cos(theta)

        dx = qx - pos[0]
        dy = qy - pos[1]

        fVM = self.factor * math.hypot(X, Y)
        fVD = math.atan2(dy, dx)

        fVX = fVM * math.cos(fVD)
        fVY = fVM * math.sin(fVD)

        return (fVX, fVY)

    # def forceAttract(self, pos):
    #     """
    #     An attractor proportional to the distance.
    #     """
    #     X = self.nodePos[0] - pos[0]
    #     Y = self.nodePos[1] - pos[1]

    #     fVM = self.factor * math.hypot(X, Y)
    #     fVD = math.atan2(Y, X)

    #     fVX = fVM * math.cos(fVD)
    #     fVY = fVM * math.sin(fVD)

    #     return (fVX, fVY)

class ForceList(object):
    """
    This class is used to manage a collection of Force objects acting on
    some robot.  The Force objects are defined and added individually,
    but the force acting on any robot is returned with the sumForces()
    method.

    The class also provides some facility for managing the list, with
    addForce() and removeForce() methods.  You can also change a Force
    with the updateForce() method, though the only change it really
    supports is to modify the Force object's node position.  For more
    elaborate modifications, you should remove the Force object from the
    list and add a new one.
    """
    def __init__(self, debug=False):
        self.flist = {}
        self.nlist = 0

    def getForce(self, index):
        return self.flist[index]

    def addForce(self, newForce):
        self.nlist += 1
        self.flist[self.nlist] = newForce
        return self.nlist

    def removeForce(self, index):
        if index in self.flist.keys():
            del self.flist[index]
            return True
        else:
            return False

    def purgeForces(self):
        """
        Removes forces from the list that are past their expiration.
        """
        tnow = rospy.Time.now().to_sec()
        itemsToDelete = []
        for i in self.flist.keys():
            if self.flist[i].expiry < tnow:
                itemsToDelete.append(i)

        itemsToDelete.sort(reverse=True)

        for k in itemsToDelete:
            del self.flist[k]


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

    def sumForces(self, pos):
        """
        Provides the sum of the forces in the list on position pos.
        """
        if self.flist:
            corrList = [ f.force(pos) for f in self.flist.values() ]

            XForce = sum([ c[0] for c in corrList ])
            YForce = sum([ c[1] for c in corrList ])

            return [XForce, YForce]
        else:
            return [0.0, 0.0]

if __name__ == '__main__':
# Testing

    import unittest
    class TestForces(unittest.TestCase):
        def setUp(self):
            rospy.init_node('gnav')
            self.fl = ForceList()
            self.ffa = ForceRepel((0.25,0.0))
            self.ffb = ForceRepel((0.25,0.25))
            self.ffc = ForceRepel((-0.25,-0.25))
            self.ffd = ForceRepel((-0.25,0.0))

            self.fva = ForceAttract((1.0, 1.0, 0.7), factor=20.0)
            self.fvb = ForceAttract((-1.0, -1.0, 0.7), factor=20.0)

            self.tiny = 0.000000001

        def testStackOperations(self):


            self.fl.addForce(self.ffa)
            t =  self.fl.sumForces((0.0,0.0,0.0))
            self.assertEqual(t[0], -256.0)
            self.assertTrue(math.fabs(t[1]) < self.tiny)


            self.fl.addForce(self.ffd)
            testString = "F1: FR:(0.250, 0.000)\nF2: FR:(-0.250, 0.000)"
            self.assertEqual(testString, self.fl.str())
            t =  self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0]) < self.tiny)
            self.assertTrue(math.fabs(t[1]) < self.tiny)

            self.fl.addForce(self.ffc)
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[1] - t[0]) < self.tiny)

            self.fl.addForce(self.ffb)
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0]) < self.tiny)
            self.assertTrue(math.fabs(t[1]) < self.tiny)

            self.fl.addForce(self.fva)
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0] + 20.0) < self.tiny)
            self.assertTrue(math.fabs(t[1] + 20.0) < self.tiny)

            self.fl.addForce(self.fvb)
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0]) < self.tiny)
            self.assertTrue(math.fabs(t[1]) < self.tiny)


            t = self.fl.getForce(2).str()
            self.assertEqual(t, "FR:(-0.250, 0.000)")

            self.assertTrue(self.fl.updateForce(5,(2.0, 2.0, 3.1416)))
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0] + 20.0) < self.tiny)
            self.assertTrue(math.fabs(t[1] + 20.0) < self.tiny)

            self.assertTrue(self.fl.removeForce(1))
            self.assertFalse(self.fl.removeForce(32))

        def testPurge(self):

            self.fl.addForce(self.ffa)
            self.fl.addForce(self.ffb)
            self.fl.addForce(self.ffc)
            self.fl.addForce(self.ffd)

            ft1 = ForceRepel((0.0,1.0), duration=2.0)
            ft2 = ForceRepel((0.0,-1.0), duration=1.0)

            self.fl.addForce(ft1)
            self.fl.addForce(ft2)
            self.fl.purgeForces()
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0]) < self.tiny)
            self.assertTrue(math.fabs(t[1]) < self.tiny)

            rospy.sleep(1.0)
            self.fl.purgeForces()
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0]) < self.tiny)
            self.assertTrue(math.fabs(t[1] + 1.0) < self.tiny)

            rospy.sleep(1.0)
            self.fl.purgeForces()
            t = self.fl.sumForces((0.0,0.0,0.0))
            self.assertTrue(math.fabs(t[0]) < self.tiny)
            self.assertTrue(math.fabs(t[1]) < self.tiny)
            

    suite = unittest.TestLoader().loadTestsFromTestCase(TestForces)
    unittest.TextTestRunner(verbosity=3).run(suite)


