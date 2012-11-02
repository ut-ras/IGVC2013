#!/usr/bin/env python
#
# Classes for making the map guesser into a server.  The filter's job
# is to accept a Guess request from perfesser and reply with a Guess where
# all the impossible points (inside walls, etc) have been discarded.
#
import roslib ; roslib.load_manifest('map_filter')
import rospy
from perfesser.srv import GuessRequest, GuessResponse
from map_guesser import *

class MapServer(object):
    """
    Class to help manage a map filter server.

    name - The name must be the name by which the Guesser services is
           known.  The same string will be used in the
           rospy.ServiceProxy call.
    nPoints - The number of points we're passing around.
    nbins = A pair with the number of bins in each x and y
    periods - We're keeping this general-ish to maintain the
           possibility of extending to three dimensions.
    announce - A publisher with which this server will anounce when it
           has something to say to the perfesser.  (i.e. Which is only when
           we have relevant map data.)
    """
    def __init__(self, name, nPoints, nBins, announce=False):
        self.name = name
        self.nPoints = nPoints
        self.nBins = nBins
        self.announce = announce

        self.guesser = MapGuesser(self.name, self.nPoints, self.nBins,
                                  announce=self.announce)
        self.ready_to_publish = False


    def answer_server(self, req):
        gr = GuessResponse()
        gr.sender = self.name

        if not self.ready_to_publish:
            gr.no_data = True
            return gr

        gr.no_data = False
        self.guesser.handle_guess(req.points)

        gr.outPoints = self.guesser.pointList
        return gr

    def addMap(self, mapData):
        self.guesser.addMap(mapData)
        self.ready_to_publish = True

        return True


if __name__ == '__main__':

    import sys
    import unittest

    class TestMapServer(unittest.TestCase):
        def setUp(self):
            self.ms = MapServer('map_filter', 1000, (10, 10))
            
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

            self.ms.addMap(mapData)

        def testAnswerServer(self):
            # Create a uniform random field
            g = Guesser("map_filter_test", self.ms.nPoints, (0.0, 0.0, 2*math.pi),
                        "location", (10,10,1))
            g.uniform([[-5.0,5.0],[-5.0,5.0],[0.0,3.0]])

            # Compose a guess request
            greq = GuessRequest()
            greq.points = g.outPoints()
            greq.pers = (0.0, 0.0, 2*math.pi)
            greq.data_type = "location"

            gresp = self.ms.answer_server(greq)
            parray = np.array([p.point for p in gresp.outPoints])

            # There should be no points left where the mapData array was 1.0
            xmax = False ; ymin = False ; xyint = False

            mx = map(max, parray.transpose())
            self.assertTrue(mx[0] < 2.5 and mx[1] < 2.5)
            mn = map(min, parray.transpose())
            self.assertTrue(mn[0] > -3.5 and mn[1] > -3.5)

            cornerTest = False
            npts = parray.shape[0]
            for i in range(npts):
                if parray[i][0] > 0.25 and \
                        parray[i][1] < -0.5:
                    cornerTest = True
            self.assertFalse(cornerTest)  

    suite = unittest.TestLoader().loadTestsFromTestCase(TestMapServer)
    unittest.TextTestRunner(verbosity=3).run(suite)
