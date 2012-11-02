#!/usr/bin/env python
#
# These classes are used to create "guesses" out of map data.
# See the perfesser guessing routines for more about guessing.
#
# The job of the map filter is to accept a hypothesis about a robot's
# location, in the form of a suite of equally  probable points, and to
# filter that collection of points using whatever we know about the
# map we're in.
import roslib ; roslib.load_manifest('map_filter')
import rospy
from perfesser_guesser import Guesser, Histogram
from perfesser.srv import Guess, GuessRequest, GuessResponse
from perfesser.msg import Pt, Belief, Announce
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class MapData(object):
    """
    Class to hold a map, a 2-d array of data indicating free space (0.0)
    and occupied space (1.0).

    mapArray - a 2-d np-style array of map points
    orig - a tuple with the coordinates of the bottom left (southwest)
           corner.
    ints - a 2-tuple containing the grid spacing in the x and y directions.
    """
    def __init__(self, mapArray, orig, ints):
        self.mapArray = mapArray
        self.orig = orig
        self.ints = ints

class MapGuesser(object):
    def __init__(self, name, nPoints, nBins, announce=False):
        self.name = name
        self.nBins = nBins
        self.nPoints = nPoints
        self.announce = announce

        self.mapData = MapData(np.array([[0.0,0.0]]), (0.0,0.0), (1.0,1.0))
        self.guesser = Guesser(self.name, (0.0, 0.0, 2*math.pi),
                               "location", 
                               (self.nBins[0],self.nBins[1],1))


    def addMap(self, newMapData):
        assert isinstance(newMapData, MapData)

        self.mapData = newMapData

        # Pack the map data into a list of Point objects, since that's
        # what the perfesser wants for input.  This will make the grid
        # into a histogram with which to filter other input points.
        self.mapPointList = []
        for xi in range(self.mapData.mapArray.shape[0]):
            for yi in range(self.mapData.mapArray.shape[1]):
                if self.mapData.mapArray[xi,yi] < 0.5:
                    p = Pt()
                    p.point = (self.mapData.orig[0] + xi * self.mapData.ints[0],
                               self.mapData.orig[1] + yi * self.mapData.ints[1],
                               0.0)
                    self.mapPointList.append(p)

    def handle_guess(self, greq):
        """
        Takes an input guess (a list of Point objects) and throws out
        the points that seem impossible.  Then resamples to return a list
        of equally probable points.  Most of this functionality is provided
        via the perfesser's guesser methods.
        """
        # Check input types
        assert isinstance(greq, GuessRequest)
        assert isinstance(greq.inPoints, list)
        assert isinstance(greq.inPoints[0], Pt)

        self.guesser.newPoints(self.mapPointList)

        pts = Belief()
        pts.points = greq.inPoints
        self.guesser.update(pts)

        # Return
        gresp = GuessResponse(sender = self.name,
                              source_stamp = rospy.Time.now(),
                              source_data = "",
                              outPoints = self.guesser.outPoints(),
                              no_data = False)

        return gresp

if __name__ == '__main__':

    rospy.init_node('map_filter')
    import sys
    import unittest
    class TestMapGuesser(unittest.TestCase):
        def setUp(self):
            self.mg = MapGuesser("map_filter", 1000, (10,10))

            mapArray = np.zeros((20,20))
            for xi in range(20):
                for yi in range(20):
                    if xi < 3 or xi > 15:
                        mapArray[xi,yi] = 1.0
                    if yi < 3 or yi > 15:
                        mapArray[xi,yi] = 1.0
                    if yi < 10 and xi > 10:
                        mapArray[xi,yi] = 1.0

            mapData = MapData(mapArray, (-5.0, -5.0), (0.5, 0.5))

            self.mg.addMap(mapData)

        def testUpdate(self):
            # Create a uniform random field
            g = Guesser("map_filter_test", (0.0, 0.0, 2*math.pi),
                        "location", (10,10,1))
            g.uniform(self.mg.nPoints, [[-5.0,5.0],[-5.0,5.0],[0.0,3.0]])

            # Use it as a guess against the filter already in mg

            gr = GuessRequest(inPoints=g.outPoints(),
                              means=g.means(),
                              stds=g.stds(),
                              data_type=g.data_type,
                              pers=g.periods)

            gresp = self.mg.handle_guess(gr)

            # There should be no points left where the mapData array was 1.0
            xmax = False ; ymin = False ; xyint = False

            xvec = [ p.point[0] for p in gresp.outPoints ]
            yvec = [ p.point[1] for p in gresp.outPoints ]

            print ">>>", max(xvec), min(xvec)
            print ">>>", max(yvec), min(yvec)




            mx = map(max, self.mg.guesser.pointArray.transpose())
            print ">>>>", mx
            self.assertTrue(mx[0] < 2.5 and mx[1] < 2.5)
            mn = map(min, self.mg.guesser.pointArray.transpose())
            self.assertTrue(mn[0] > -3.5 and mn[1] > -3.5)

            cornerTest = False
            npts = self.mg.guesser.pointArray.shape[0]
            for i in range(npts):
                if self.mg.guesser.pointArray[i][0] > 0.25 and \
                        self.mg.guesser.pointArray[i][1] < -0.5:
                    cornerTest = True
            self.assertFalse(cornerTest)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestMapGuesser)
    unittest.TextTestRunner(verbosity=3).run(suite)


    if len(sys.argv) > 1:

        mg = MapGuesser("map_filter", 1000, (10,10))

        mapArray = np.zeros((20,20))
        for xi in range(20):
            for yi in range(20):
                if xi < 3 or xi > 15:
                    mapArray[xi,yi] = 1.0
                if yi < 3 or yi > 15:
                    mapArray[xi,yi] = 1.0
                if yi < 10 and xi > 10:
                    mapArray[xi,yi] = 1.0


        mapData = MapData(mapArray, (-5.0, -5.0), (0.5, 0.5))

        mg.addMap(mapData)

        import tgraph
        tg = tgraph.Tgraph(350,200)

        inarray = np.array([[-5.5,-5.5,0.0],[5.5,5.5,0.6]])
        tg.draw_scatter(inarray, 0,1,2, "s")

        parray = np.array([p.point for p in mg.mapPointList])
        tg.plot_points(parray, 0,1,2, "s")

        g = Guesser("map_filter_test", (0.0, 0.0, 2*math.pi),
                    "location", (10,10,1))

        g.uniform(mg.nPoints, [[-5.0,5.0],[-5.0,5.0],[0.0,3.0]])

        tg.new_graph()
        tg.draw_scatter(g.pointArray, 0,1,2, "s", recalc=False)

        gr = GuessRequest(inPoints=g.outPoints(),
                          means=g.means(),
                          stds=g.stds(),
                          data_type=g.data_type,
                          pers=g.periods)

        gresp = mg.handle_guess(gr)

        tg.plot_points(mg.guesser.hist.plottable(), 0,1,2,"c")

        tg.new_graph()
        parray = np.array([p.point for p in gresp.outPoints])
        tg.draw_scatter(parray, 0,1,2, "s", recalc=False)

        tg.mainloop()

