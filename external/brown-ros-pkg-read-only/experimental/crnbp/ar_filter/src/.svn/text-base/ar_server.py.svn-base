#!/usr/bin/env python
#
# Some classes for making the ar guesser into a server.  The filter's
# job is to accept a Guess request from perfesser and reply with the
# points of the request added to whatever we can glean from an
# observed AR tag.
#
import roslib ; roslib.load_manifest('ar_filter')
import rospy
from perfesser.srv import GuessRequest, GuessResponse
import math
from ar_guesser import *
# debug only
from std_msgs.msg import String


class ARServer(object):
    """
    A class to help manage an AR guess server.

    name - The name must be the name by which the Guesser services is
           known.  The same string will be used in the
           rospy.ServiceProxy call.
    nPoints - The number of points we're passing around.
    periods - We're keeping this general-ish to maintain the
           possibility of extending to three dimensions.
    idError, dError, angError - Refer to the ar_guesser.py comments.
    announce - A publisher with which this server will anounce when it
           has something to say to the perfesser.  (i.e. When we have
           a tag in view.)
    lfile - A file with tag id numbers and locations.
    """
    def __init__(self, name, nPoints, periods, dError, angError,
                 idError, motionError, announce=False, landmarks=False,
                 locfile=""):
        self.name = name
        self.nPoints = nPoints
        self.periods = periods
        self.idError = idError
        self.dError = dError
        self.angError = angError
        self.motionError = motionError
        self.announce = announce
        self.landmarks = landmarks

        self.arGuesser = ARGuesser(self.name, self.nPoints,
                                   self.dError, self.angError, self.idError,
                                   self.motionError,
                                   announce=self.announce,
                                   landmarks=self.landmarks)

        if locfile:
            with open(locfile, 'r') as locationFile:
                for line in locationFile:
                    if line.startswith("#"):
                        continue
                    tagInfo = line.split()
                    if len(tagInfo) != 4:
                        continue
                    self.arGuesser.addTag(int(tagInfo[0]),
                                        (float(tagInfo[1]),
                                         float(tagInfo[2]),
                                         float(tagInfo[3])))
        self.ready = False

        self.pdebug = rospy.Publisher("/guesser/position/debug", String)


    def answer_server(self, req):

        # If the previous guess was also ar-based, dump it without filtering
        # it against the current guess.
#        if req.source_data[0:3] == "artg":
#            gr.no_data = False
#            gr.source_stamp = self.arGuesser.stamp
#            gr.source_data = self.arGuesser.source_data
#            gr.outPoints = self.arGuesser.pointList
        # Filter the input points against the AR estimate.
#        elif self.arGuesser.handle_guess(req.points):
        gresp = self.arGuesser.handle_guess(req)

        # debug:
        s = String()
        s.data = "Inside ar_server: \n"
        s.data += "input: %s\n" % req.source_data
        s.data += "output: %s\n" % gresp.source_data
        self.pdebug.publish(s)

        return gresp

if __name__ == '__main__':

    import sys
    import unittest
    rospy.init_node("ar_filter")

    class TestARServer(unittest.TestCase):
        def setUp(self):
            self.tmpfile = "/tmp/trash.txt"
            with open(self.tmpfile, "w") as testfile:
                testfile.write("1 1.0 1.0 -1.552\n")
                testfile.write("2 1.0 -1.0 1.552\n")
                testfile.write("3 -1.0 -1.0 -1.552\n")
                testfile.write("4 -1.0 1.0 -1.552\n")

        def testService(self):

            ars = ARServer("ar_filter", 1000, (0.0, 0.0, 2*math.pi), 0.1, 0.02,
                           .95, 3.0, locfile=self.tmpfile)


            ts = Tags()
            t = Tag()
            t.id = 1
            t.cf = 0.75
            t.x = 1.0
            t.y = 1.0
            t.distance = 2.0**0.5
            t.xRot = 0.0
            t.yRot = math.pi/4.0
            t.zRot = 0.0
            t.xMetric = 1.000
            t.yMetric = 1.000
            t.zMetric = 0
            ts.tags.append(t)
            ts.tag_count = 1

            ars.arGuesser.handle_image(ts)

            g = Guesser("ar_filter_test", (0.0, 0.0, 2*math.pi),
                        "location", (1,1,1))

            xmax = 2.5 ; ymin = 0.0
            g.uniform(ars.nPoints, [[0.0,xmax],[ymin,2.0],[0.0,3.25]])

            greq = GuessRequest()
            greq.inPoints = g.outPoints()
            greq.pers = (0.0, 0.0, 2*math.pi)
            greq.data_type = "location"

            gr = ars.answer_server(greq)
            parray = np.array([p.point for p in gr.outPoints])

            mins = map(min, parray.transpose())
            maxs = map(max, parray.transpose())

            #print maxs, mins, xmax, ymin
            self.assertTrue(mins[1] > ymin)
            self.assertTrue(maxs[0] < xmax)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestARServer)
    unittest.TextTestRunner(verbosity=3).run(suite)

    if len(sys.argv) > 1:
        tmpfile = "/tmp/trash.txt"
        with open(tmpfile, "w") as testfile:
            testfile.write("1 1.0 1.0 -1.552\n")
            testfile.write("2 1.0 -1.0 1.552\n")
            testfile.write("3 -1.0 -1.0 -1.552\n")
            testfile.write("4 -1.0 1.0 -1.552\n")


        ars = ARServer("ar_filter", 1000, (0.0, 0.0, 2*math.pi), 0.1, 0.02,
                       .95, 1.0, locfile=tmpfile)


        ts = Tags()
        t = Tag()
        t.id = 1
        t.cf = 0.75
        t.x = 1.0
        t.y = 1.0
        t.distance = 2.0**0.5
        t.xRot = 0.0
        t.yRot = math.pi/4.0
        t.zRot = 0.0
        t.xMetric = 1.000
        t.yMetric = 1.000
        t.zMetric = 0
        ts.tags.append(t)
        ts.tag_count = 1

        ars.arGuesser.handle_image(ts)

        g = Guesser("ar_filter_test", (0.0, 0.0, 2*math.pi),
                    "location", (1,1,1))

        g.uniform(ars.nPoints, [[0.0,2.25],[0.0,2.0],[0.0,3.25]])

        greq = GuessRequest()
        greq.inPoints = g.outPoints()
        greq.pers = (0.0, 0.0, 2*math.pi)
        greq.data_type = "location"

        import tgraph
        tg = tgraph.Tgraph(400,300)

        parray = np.array([p.point for p in greq.inPoints])
        tg.draw_scatter(parray, 0,1,2, "s")

        gr = ars.answer_server(greq)

        tg.new_graph()
        parray = np.array([p.point for p in ars.arGuesser.tarPts.points])
        tg.draw_scatter(parray, 0,1,2, "s", recalc=False)


        tg.new_graph()
        parray = np.array([p.point for p in gr.outPoints])
        tg.draw_scatter(parray, 0,1,2, "s", recalc=False)

        tg.mainloop()

