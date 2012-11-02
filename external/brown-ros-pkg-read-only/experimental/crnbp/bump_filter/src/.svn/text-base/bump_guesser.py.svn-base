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
from perfesser_guesser import Guesser, HistogramData
from perfesser.msg import Point, Belief, Announce
from irobot_create_2_1a.msg import SensorPacket
from nav_msgs.msg import OccupancyGrid
from map_data import *
import numpy as np
import random
import math

class BumpGuesser(object):
    def __init__(self, name, nPoints, nBins, wallError, announce=False):
        self.name = name
        self.nBins = nBins
        self.nPoints = nPoints
        # wallError is the probability that a bump is from a feature that
        # is on the map.  You can bump into moving objects, too.
        self.wallError = wallError
        self.announce = announce

        self.mapData = OccupancyGrid()
        self.guesser = Guesser(self.name, self.nPoints, (0.0, 0.0, 2*math.pi),
                               "location",
                               HistogramData((self.nBins[0],self.nBins[1],1)))

        self.ready_to_publish = False

        self.stamp = rospy.Time.now()

    def feltBump(self):
        """
        Use this to signal that we've felt a bump and will shortly be contacted
        about using it for a guess.
        """
        self.stamp = rospy.Time.now()
        self.ready_to_publish = True

    def occupiedNeighbor(self, xi, yi):
        """
        Returns True if one of the immediate neighbor cells is
        occupied.  You shouldn't expect good results if you call this
        on an occupied cell.
        """

        xmax = self.mapData.og.info.width
        ymax = self.mapData.og.info.height

        if self.mapData.sampled:
            # Fails on an occupied cell
            assert self.mapData.mapArrayS[xi, yi] < 50
            for x in range(max(xi - 1, 0), min(xi + 1, xmax)):
                for y in range(max(yi - 1, 0), min(yi + 1, ymax)):
                    if self.mapData.mapArrayS[x,y] > 50:
                        return True
            return False
        else:
            # Fails on an occupied cell
            assert self.mapData.mapArray[xi, yi] < 50
            for x in range(max(xi - 1, 0), min(xi + 1, xmax)):
                for y in range(max(yi - 1, 0), min(yi + 1, ymax)):
                    if self.mapData.mapArray[x,y] > 50:
                        return True
            return False

    def addMap(self, newMapData):
        """ Receives a new map. """
        assert isinstance(newMapData, MapData)

        self.mapData = newMapData

    def handle_guess(self, newPointList):
        """
        Takes an input guess (a list of Point objects) and filters it
        against a map of points that seem probable because of a recent
        bump.  Then resamples per usual to return a list of equally
        probable points.
        """
        if not self.ready_to_publish:
            return False
        assert isinstance(newPointList, list)
        assert isinstance(newPointList[0], Point)

        # Find the limits of the input data.
        xmax = -1.0e6 ; ymax = -1.0e6
        xmin = 1.0e6 ; ymin = 1.0e6
        for pt in newPointList:
            xmax = max(xmax, pt.point[0])
            ymax = max(ymax, pt.point[1])
            xmin = min(xmin, pt.point[0])
            ymin = min(ymin, pt.point[1])

        # Shrink the map to accommodate the relevant area
        self.mapData.sample((xmin,ymin), (xmax,ymax))

        # Cruise through the map looking for empty cells next to occupied
        # ones.  These will be the likely cells when a bump is encountered.
        #
        # Because of the possibility of bumping an object that isn't on the
        # map, any empty map cell is possible.  Therefore, we run through
        # the map, packing the map data into a list of Point objects, since
        # that's what the perfesser wants for input.  While we're running
        # through, we keep a separate list of empty cells next to full ones.
        wallPointList = []
        emptyPointList = []
        for xi in range(self.mapData.ogS.info.width):
            for yi in range(self.mapData.ogS.info.height):
                if self.mapData.mapArrayS[xi,yi] < 50:
                    p = Point()
                    p.point = self.mapData.transform((xi, yi))
                    emptyPointList.append(p)
                    if (self.occupiedNeighbor(xi, yi)):
                        newP = Point()
                        newP.point = (p.point[0] + np.random.normal(0.0, self.mapData.ogS.info.resolution/3.0),
                                      p.point[1] + np.random.normal(0.0, self.mapData.ogS.info.resolution/3.0),
                                      p.point[2])

                        wallPointList.append(newP)

        # Using the wallError, sample the two lists together to get a roughly
        # correct distribution of points to feed to the perfesser.
        self.mapPointList = []
        for i in range(self.nPoints):
            if i < self.wallError * self.nPoints:
                self.mapPointList.append(random.choice(wallPointList))
            else:
                self.mapPointList.append(random.choice(emptyPointList))

        self.guesser.newPoints(self.mapPointList)

        pts = Belief()
        pts.points = newPointList
        self.guesser.update(pts)

        self.pointList = self.guesser.outPoints()
        self.ready_to_publish = False
        return True

if __name__ == '__main__':

    import sys
    import unittest
    class TestBumpGuesser(unittest.TestCase):
        def setUp(self):
            self.bg = BumpGuesser("bump_filter", 1000, (10,10), 0.8)

            og = OccupancyGrid()
            og.info.resolution = 0.5
            og.info.height = 20
            og.info.width = 20
            og.info.origin.position.x = 0.0
            og.info.origin.position.y = 0.0
            og.info.origin.orientation.z = 0.0#5 * 2.0**0.5
            og.info.origin.orientation.w = 0.0#5 * 2.0**0.5
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

            mapData = MapData(og)
            self.bg. addMap(mapData)

        def testOccupied(self):
            self.assertTrue(self.bg.occupiedNeighbor(3,4))
            self.assertFalse(self.bg.occupiedNeighbor(6,6))
            self.assertRaises(AssertionError, self.bg.occupiedNeighbor, 0, 0)

        def testUpdate(self):

            # Create a uniform random field
            g = Guesser("map_filter_test", self.bg.nPoints, (0.0, 0.0, 2*math.pi),
                         "location", HistogramData((10,10,1)))
            g.uniform([[-5.0,5.0],[-5.0,5.0],[0.0,3.0]])

            # # Use it as a guess against the filter already in mg
            # self.mg.handle_guess(g.outPoints())

            # # There should be no points left where the mapData array was 1.0
            # xmax = False ; ymin = False ; xyint = False

            # mx = map(max, self.mg.guesser.pointArray.transpose())
            # self.assertTrue(mx[0] < 2.5 and mx[1] < 2.5)
            # mn = map(min, self.mg.guesser.pointArray.transpose())
            # self.assertTrue(mn[0] > -3.5 and mn[1] > -3.5)

            # cornerTest = False
            # npts = self.mg.guesser.pointArray.shape[0]
            # for i in range(npts):
            #     if self.mg.guesser.pointArray[i][0] > 0.25 and \
            #             self.mg.guesser.pointArray[i][1] < -0.5:
            #         cornerTest = True
            # self.assertFalse(cornerTest)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestBumpGuesser)
    unittest.TextTestRunner(verbosity=3).run(suite)


    if len(sys.argv) > 1:

        bg = BumpGuesser("bump_filter", 1000, (10,10), 0.99)

        og = OccupancyGrid()
        og.info.resolution = 0.5
        og.info.height = 24
        og.info.width = 24
        og.info.origin.position.x = -6.0
        og.info.origin.position.y = -6.0
        og.info.origin.orientation.z = 0.0#5 * 2.0**0.5
        og.info.origin.orientation.w = 0.0#5 * 2.0**0.5
        og.info.origin.orientation.x = 0.0
        og.info.origin.orientation.y = 0.0

        mapArray = np.array([0] * 576)
        mapArray.shape = (og.info.height, og.info.width)

        for xi in range(20):
            for yi in range(20):
                if xi < 3 or xi > 15:
                    mapArray[xi,yi] = 100
                if yi < 3 or yi > 15:
                    mapArray[xi,yi] = 100
                if yi < 10 and xi > 10:
                    mapArray[xi,yi] = 75

        mapArray.shape = 576
        og.data = list(mapArray)

        mapData = MapData(og)
        bg.addMap(mapData)

        import tgraph
        tg = tgraph.Tgraph(350, 350)

        g = Guesser("map_filter_test", bg.nPoints, (0.0, 0.0, 2*math.pi),
                    "location", HistogramData((10,10,1)))
        g.uniform([[-5.0,5.0],[-5.0,5.0],[0.0,3.0]])

        bg.handle_guess(g.outPoints())

        plist = []
        for pt in bg.mapPointList:
            plist.append(pt.point)

        tg.draw_scatter(np.array([[-5.0, -5.0, 0.0], [5.0, 5.0, 2.0]]), 0,1,2,"s")
        tg.plot_points(np.array(plist), 0,1,2, "s")

        tg.mainloop()

