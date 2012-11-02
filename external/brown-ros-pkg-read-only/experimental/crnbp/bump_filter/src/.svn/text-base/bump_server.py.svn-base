#!/usr/bin/env python
#
# Some classes for making the bump guesser into a server.  The filter's
# job is to accept a Guess request from perfesser and reply with the
# points of the request added to whatever we can glean from an
# observed bump.
#
import roslib ; roslib.load_manifest('bump_filter')
import rospy
from irobot_create_2_1a.msg import SensorPacket
from perfesser.msg import Announce
from perfesser.srv import GuessRequest, GuessResponse
from step_nav.msg import Perturb
import math
#from bump_guesser import *


class BumpGuesser(object):
    """
    A dummy class for the moment.  Using this defeats the map-guessing
    capacity of the bump filter, but leaves the bump-handling function.
    """
    def __init__(self, name, nPoints, nBins, wallError):
        self.name = name
        self.nPoints = nPoints
        self.nBins = nBins
        self.wallError = wallError

    def handle_guess(self, req):
        return False

    def addMap(self, maps):
        return

    def feltBump(self):
        return

class BumpServer(object):
    """
    A class to help manage a bump guess server.  This class takes care
    of notifying the path planner that a bump has happened and that
    some action is required.  It also dispatches a call to
    bump_guesser to see if we can glean any location information from
    the event.

    name - The name must be the name by which the Guesser services is
           known.  The same string will be used in the
           rospy.ServiceProxy call.
    nPoints - The number of points we're passing around.
    wallError - Refer to the bump_guesser.py comments.
    announce - A publisher with which this server will anounce when it
           has something to say to the perfesser.  (i.e. When we bumped.)
    """
    def __init__(self, name, nPoints, nBins, wallError, announce=False, 
                 perturb=False):
        self.name = name
        self.nPoints = nPoints
        self.nBins = nBins
        self.wallError = wallError
        self.announce = announce

        self.guesser = BumpGuesser(self.name, self.nPoints, 
                                   self.nBins, self.wallError)
        # This flag is used to make sure we only react to one bump, even 
        # though it may be reported multiple times.
        self.bumped = False

        # This is the publisher we use to signal that a perturbation of
        # the path is in order so we can deal with a bump.
        self.perturb = perturb

    def answer_server(self, req):
        """
        Provides a reply to a Guess service call.  The Perfesser will call 
        this after we've notified him that we have something worth saying
        about the robot's location.  The call is simply passed into the
        bump_guess Guesser.
        """
        gr = GuessResponse()
        gr.sender = self.name

        if self.guesser.handle_guess(req.inPoints):
            gr.no_data = False
            gr.outPoints = self.guesser.pointList
            gr.source_stamp = self.guesser.stamp
            gr.source_data = self.guesser.source_data
        else:
            gr.no_data = True
            return gr
            

    def addMap(self, req):
        """
        Receives an OccupancyGrid object, converts it to a MapData object
        and passes it into the guesser.
        """
        self.guesser.addMap(MapData(req))

    def checkSensorPacket(self, req):
        """
        Receives the published sensorPacket and looks for a bump.  If
        there is one we signal the guesser routine, and also publish
        on the announce topic that we've got something to tell the
        world out there about our location.
        """
        # Check to see if we've bumped, and that it isn't an old bump.
        if req.bumpLeft or req.bumpRight:
            if self.bumped:
                # We've already heard about this one
                pass
            else:
                self.guesser.feltBump()
                self.bumped = True
                if self.announce:
                    a = Announce()
                    a.sender = self.name
                    self.announce.publish(a)
        else:
            # If there is no longer a bump detected, clear the bumped flag.
            self.bumped = False
        
        return
