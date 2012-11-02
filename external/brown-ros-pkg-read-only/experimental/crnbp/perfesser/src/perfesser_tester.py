#!/usr/bin/env python
from perfesser.msg import Pt, Belief
import sys

class TestPublisher(object):
    def __init__(self):
        self.output = Belief()

    def publish(self,x):
        self.output = x

    def testObject(self):
        """ Testing the correct object is sent to the publisher."""
        try:
            out = ""
            out += "%s\n" % str(self.output.header)
            out += "%s\n" % str(self.output.data_type)
            out += "%s\n" % str(self.output.sender)
            out += "%s\n" % str(self.output.means)
            out += "%s\n" % str(self.output.stds)
            out += "%s\n" % str(self.output.pers)
            out += "%s\n" % str(self.output.source_stamp)
            out += "%s\n" % str(self.output.source_data)
            out += "%s\n" % str(self.output.points)
            out += "%s\n" % str(self.output.points[0].point)
            return True
        except:
            print str(sys.exc_info())
            return False
