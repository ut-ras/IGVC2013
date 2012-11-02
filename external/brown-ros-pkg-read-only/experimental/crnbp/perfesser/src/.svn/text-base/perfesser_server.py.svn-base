#!/usr/bin/env python
#
# A lightweight class to manage the perfesser server.  Maintains a stack of
# service proxies we're going to call.  Operation proceeds by popping
# the top entry off the stack and calling it.
#
import roslib ; roslib.load_manifest('perfesser')
import rospy
from perfesser_guesser import *
from perfesser.srv import Guess, GuessRequest, GuessResponse
# debugging
from std_msgs.msg import String
import math

class PerfesserServer(object):
    """
    The way the Perfesser works is that filter nodes who have data to offer publish a message to say so.  Then, in turn, the Perfesser calls on them with its query.


    """


    def __init__(self, servicefn, debug=False):
        # This is the stack.
        self.service_stack = []
        self.services = {}
        # This will usually be rospy.ServiceProxy, but is made
        # replaceable for testing purposes.
        self.servicefn = servicefn
        self.counter = 0
        self.debug = debug

        # debugging
        self.pdebug = rospy.Publisher('guesser/position/debug', String)

    def surveyGuess(self, guesser):
        if self.service_stack:
            s = self.service_stack.pop(0)

            g = GuessRequest()
            g.inPoints = guesser.outPoints()
            g.means = guesser.means()
            g.stds = guesser.stds()
            g.pers = guesser.periods
            g.data_type = guesser.data_type
            g.source_stamp = guesser.stamp
            g.source_data = guesser.source_data

            service = self.services[s]
            resp = service(g)

            if self.debug:
                s = String()
                self.counter += 1
                s.data += " CALLED:%d:%s \n" % (self.counter, str(service),)
                s.data += str(service.resolved_name) + " "
                s.data += resp.source_data + "\n"
                s.data += str(resp.source_stamp.secs) + ":" + str(resp.source_stamp.nsecs)
                self.pdebug.publish(s)

            if not resp.no_data:
                guesser.newPoints(resp.outPoints)
                guesser.stamp = resp.source_stamp
                guesser.source_data = resp.source_data
        else:
            resp = GuessResponse()
            resp.no_data = True

        return resp

    def updateStack(self, req):
        sender = req.sender

        if not sender in self.services.keys():
            self.services[sender] = self.servicefn(sender, Guess, 
                                                   persistent=True)

        if not sender in self.service_stack:
            self.service_stack.append(sender)

if __name__ == '__main__':

    class TestAnnounce(object):
        def __init__(self, sender):
            self.sender = sender
            self.no_data = True

    class TestService(object):
        def __init__(self, sender):
            self.sender = sender

        def testService(self, guess):
            guess.data_type = ">>>%s" % (self.sender,)
            return TestAnnounce(self.sender)

    def testFn(sender, obj):
        return TestService(sender).testService

    import unittest
    class TestPerfesserServer(unittest.TestCase):
        def setUp(self):
            self.ps = PerfesserServer(testFn)

            self.ps.updateStack(TestAnnounce("entryOne"))
            self.ps.updateStack(TestAnnounce("entryTwo"))
            self.ps.updateStack(TestAnnounce("entryThree"))

            self.guesser = Guesser("TestGuesser", 10, (0.0,0.0,2*math.pi, 4.0),
                                   "pose", HistogramData((4,4,7.3)))

            self.guesser.normal([[1.0,1.0], [0.0,1.0], [0.0,1.0], [0.0,1.0]])

        def testStack(self):

            self.assertEqual(len(self.ps.service_stack), 3)
            self.assertEqual(self.ps.service_stack[1],"entryTwo")
            self.assertEqual(len(self.ps.services.keys()), 3)

            r = self.ps.surveyGuess(self.guesser)

            self.assertEqual(r.sender, "entryOne")
            self.assertEqual(self.guesser.data_type, "pose")

            return

            print self.ps.service_stack
            print self.ps.services

            self.ps.updateStack(TestAnnounce("entryTwo"))

            print self.ps.service_stack
            print self.ps.services

            self.ps.surveyGuess(self.guesser)

            self.ps.updateStack(TestAnnounce("entryFour"))

            print self.ps.service_stack
            print self.ps.services

            self.ps.surveyGuess(self.guesser)

            print self.ps.service_stack
            print self.ps.services

            self.ps.surveyGuess(self.guesser)

            print self.ps.service_stack
            print self.ps.services

    suite = unittest.TestLoader().loadTestsFromTestCase(TestPerfesserServer)
    unittest.TextTestRunner(verbosity=3).run(suite)


