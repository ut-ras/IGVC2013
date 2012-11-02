#!/usr/bin/env python
#
# These classes are used to create "guesses" out of map data and a bump.
# See the perfesser guessing routines for more about guessing.
#
# The job of the bump filter is to accept a hypothesis about a robot's
# location, in the form of a suite of equally  probable points, and to
# filter that collection of points using whatever we know about the map
# plus the fact that we bumped into something.
import roslib ; roslib.load_manifest('bump_filter')
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class MapData(object):
    """
    Class to hold a map, a 2-d array of data indicating free space (0.0)
    and occupied space (1.0).

    og = an OccupancyGrid object
    mapArray - a 2-d np-style array of map points made from og.data
    """
    def __init__(self, og):
        self.og = og
        self.mapArray = np.array(og.data)
        self.mapArray.shape = (self.og.info.width, self.og.info.height)
        self.sampled = False

    def transform(self, pt):
        """
        Transforms the indexes of the map array into real world coordinates.
        We are purposely not using tf here because of the rpc overhead.  This
        transform won't be nested, so much of tf's flexibility is not usable
        in this context and it only represents overhead.
        """
        xi = pt[0] ; yi = pt[1] ## Integers
        # Translate...
        x = self.og.info.origin.position.x + \
            xi * self.og.info.resolution
        y = self.og.info.origin.position.y + \
            yi * self.og.info.resolution
        theta = 0.0

        rz = self.og.info.origin.orientation.z
        rw = self.og.info.origin.orientation.w
        # ... then rotate.
        theta = math.fabs(2 * math.asin(rz))
        theta = -theta if (rz * rw) < 0 else theta
        
        xOut = x * math.cos(theta) + y * math.sin(theta)
        yOut = - x * math.sin(theta) + y * math.cos(theta)

        return (xOut, yOut, theta)

    def untransform(self, pt):
        """
        Returns the array indices closest to the given x and y values.
        """
        x = pt[0] ; y = pt[1] # Floats.

        rz = self.og.info.origin.orientation.z
        rw = self.og.info.origin.orientation.w
        # ... then rotate.
        theta = math.fabs(2 * math.asin(rz))
        # Reverse sign
        theta = theta if (rz * rw) < 0 else -theta

        xIn = x * math.cos(theta) + y * math.sin(theta)
        yIn =  - x * math.sin(theta) + y * math.cos(theta)

        xi = int(round((xIn - self.og.info.origin.position.x) / \
                           self.og.info.resolution))
        yi = int(round((yIn - self.og.info.origin.position.y) / \
                           self.og.info.resolution))
        
        return (xi, yi)

    def samp(self, pt, n):
        """
        Defines an n x n sample of the mapArray, centered on or near
        the given point pt.  The result is stored in mapArrayS, and 
        corresponding metadata in ogS.
        """
        # Check if we're in bounds
        llpt = self.transform((0,0))
        urpt = self.transform((self.og.info.width-1, self.og.info.height-1))

        # TODO: This should probably raise something beside an assertion error
        assert (pt[0] - urpt[0]) * (llpt[0] - pt[0]) > 0
        assert (pt[1] - urpt[1]) * (llpt[1] - pt[1]) > 0

        ctr = self.untransform(pt)
        xmax = min(ctr[0] + n/2, self.mapArray.shape[0])
        ymax = min(ctr[1] + n/2, self.mapArray.shape[1])
        xmin = max(ctr[0] - n/2, 0)
        ymin = max(ctr[1] - n/2, 0)

        self.mapArrayS = self.mapArray[xmin:xmax, ymin:ymax]

        self.ogS = OccupancyGrid()
        pt = self.transform((xmin, ymin))
        
        self.ogS.info.origin.position.x = pt[0] 
        self.ogS.info.origin.position.y = pt[1] 
        self.ogS.info.origin.orientation.z = self.og.info.origin.orientation.z
        self.ogS.info.origin.orientation.w = self.og.info.origin.orientation.w
        self.ogS.info.resolution = self.og.info.resolution
        self.ogS.info.width = xmax - xmin
        self.ogS.info.height = ymax - ymin

        self.sampled = True

    def sample(self, llpt, urpt):
        """
        Given two limiting corners of a rectangular area, creates a sample
        of the big grid of a roughly comparable area.  We want it big enough
        to cover the input area,  but not too much bigger, for computational
        efficiency reasons.
        """
        xrng = math.fabs(urpt[0] - llpt[0])
        yrng = math.fabs(urpt[1] - llpt[1])
        xn = xrng / self.og.info.resolution
        yn = xrng / self.og.info.resolution
        n = int(round(1.1 * max(xn, yn)))

        self.samp(((urpt[0] + llpt[0])/2.0, (urpt[1] + llpt[1])/2.0), n)

if __name__ == '__main__':

    import sys
    import unittest
    class TestMapData(unittest.TestCase):
        def setUp(self):

            og = OccupancyGrid()
            og.info.resolution = 0.5
            og.info.height = 20
            og.info.width = 20
            og.info.origin.position.x = 0.0
            og.info.origin.position.y = 0.0
            og.info.origin.position.y = 0.0
            og.info.origin.orientation.z = 0.0
            og.info.origin.orientation.w = 0.0
            og.info.origin.orientation.x = 0.0
            og.info.origin.orientation.y = 0.0

            mapArray = np.array([0] * 400)
            mapArray.shape = (20,20)

            for xi in range(20):
                for yi in range(20):
                    if xi < 3 or xi > 15:
                        mapArray[xi,yi] = 100
                    if yi < 3 or yi > 15:
                        mapArray[xi,yi] = 100
                    if yi < 10 and xi > 10:
                        mapArray[xi,yi] = 75

            mapArray.shape = 400
            og.data = list(mapArray)

            self.mapData = MapData(og)

        def testTransform(self):
            pt = self.mapData.transform((3,4))
            self.assertTrue(pt[0] == 1.5)
            self.assertTrue(pt[1] == 2.0)

            self.mapData.og.info.origin.orientation.z = 1.0
            self.mapData.og.info.origin.orientation.w = 1.0
            pt = self.mapData.transform((3,4))
            self.assertTrue(math.fabs(pt[0] + 1.5) < 0.000001)
            self.assertTrue(math.fabs(pt[1] + 2.0) < 0.000001)


            self.mapData.og.info.origin.position.x = 1.0
            self.mapData.og.info.origin.position.y = 2.0
            pt = self.mapData.transform((3,4))
            self.assertTrue(math.fabs(pt[0] + 2.5) < 0.000001)
            self.assertTrue(math.fabs(pt[1] + 4.0) < 0.000001)

        def testUntransform(self):
            pti = (3,4)
            pt = self.mapData.transform(pti)
            self.assertTrue(pti == self.mapData.untransform(pt))

            self.mapData.og.info.origin.orientation.z = 1.0
            self.mapData.og.info.origin.orientation.w = 1.0
            pt = self.mapData.transform(pti)
            self.assertTrue(pti == self.mapData.untransform(pt))

            self.mapData.og.info.origin.position.x = 1.0
            self.mapData.og.info.origin.position.y = 2.0
            pt = self.mapData.transform(pti)
            self.assertTrue(pti == self.mapData.untransform(pt))

            self.mapData.og.info.origin.orientation.z = 1.0
            self.mapData.og.info.origin.orientation.w = 1.0
            self.mapData.og.info.origin.position.x = 1.0
            self.mapData.og.info.origin.position.y = 2.0
            pt = self.mapData.transform(pti)
            self.assertTrue(pti == self.mapData.untransform(pt))

        def testSamp(self):
            self.mapData.og.info.origin.orientation.z = 0.5 * 2.0**0.5
            self.mapData.og.info.origin.orientation.w = 0.5 * 2.0**0.5
            self.mapData.og.info.origin.position.x = 1.0
            self.mapData.og.info.origin.position.y = 2.0

            self.mapData.samp((3.0, -8.0), 10)

            self.assertEqual(self.mapData.mapArrayS[1,2], 100)
            self.assertEqual(self.mapData.mapArrayS[1,3], 0)
            self.assertEqual(self.mapData.mapArrayS[2,2], 75)
            self.assertEqual(self.mapData.mapArrayS[2,3], 75)

            self.assertRaises(AssertionError,self.mapData.samp, (3.0, 4.0), 10)

        def testSample(self):
            self.mapData.og.info.origin.orientation.z = 0.5 * 2.0**0.5
            self.mapData.og.info.origin.orientation.w = 0.5 * 2.0**0.5
            self.mapData.og.info.origin.position.x = 1.0
            self.mapData.og.info.origin.position.y = 2.0

            self.mapData.sample((3.0,-4.0), (6.0, -9.0))

            self.assertEqual(self.mapData.mapArrayS[2,0], 100)
            self.assertEqual(self.mapData.mapArrayS[2,1], 0)
            self.assertEqual(self.mapData.mapArrayS[3,2], 75)
            self.assertEqual(self.mapData.mapArrayS[3,3], 75)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestMapData)
    unittest.TextTestRunner(verbosity=3).run(suite)
