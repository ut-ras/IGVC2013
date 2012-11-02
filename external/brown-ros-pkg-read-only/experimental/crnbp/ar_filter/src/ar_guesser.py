#!/usr/bin/env python
#
# These classes are used to create "guesses" out of AR recognition data.
# See the perfesser guessing routines for more about guessing.
#
# The job of the AR filter is to accept a hypothesis about a robot's
# location, in the form of a suite of equally probable points, and to
# filter that collection of points using whatever we know about the
# robot's location from observations of a set of AR tags whose
# location is known.
#
# This uses the ar_recog node, which in turn uses AR_Tookit.
#
import roslib ; roslib.load_manifest('ar_filter')
import rospy
from ar_recog.msg import Tag, Tags
from perfesser_guesser import Guesser
from perfesser.msg import Pt, Belief, Announce
from perfesser.srv import Guess, GuessRequest, GuessResponse
from ar_filter.msg import Lmarks, Lmark
from std_msgs.msg import String
import numpy as np
import math

class TagLoc(object):
    """
    Class to hold a priori information about a particular tag.  This
    will usually be just a tag's location.  We use this object both to
    hold the absolute values of a tag's location (x,y,theta) and the
    measured values from a Tag object (xMetric, yMetric, yRot).
    """
    def __init__(self, stamp, arg):
        self.time = stamp.to_sec()
        if isinstance(arg, Tag):
            self.loc = ( arg.xMetric, arg.yMetric, arg.yRot )
            self.dist = arg.distance
            self.tag = arg
        else:
            self.loc = arg
            self.tag = False

    def str(self):
        """
        A pretty-printer.
        """
        s = "time: %.3f\n" % (self.time,)
        s += "loc: (%.3f, %.3f, %.3f)\n" % self.loc
        s += "dist: %.3f\n" % self.dist
        if self.tag:
            s += "id: %d, dist: %d, diam: %.3f\n" % \
                (self.tag.id, self.tag.distance, self.tag.diameter)
            s += "metric: %.3f, %.3f, %.3f\n" % \
                (self.tag.xMetric, self.tag.yMetric, self.tag.zMetric)
            s += "rotate: %.3f, %.3f, %.3f\n" % \
                (self.tag.xRot, self.tag.yRot, self.tag.zRot)
        return s

class ARGuesser(object):
    def __init__(self, name, nPoints, dError, angError, idError, motionError,
                 announce=False, landmarks=False):
        """
        We receive tags messages and store the estimated tag
        locations.  When we've got one or more, we announce the fact
        and wait for a query.

        On its own schedule, the perfesser will ask us where we think
        we are.  At that point, we use the accumulated tag locations
        to filter the incoming points from the perfesser and return an
        estimate.
        """
        # Should almost always be "ar_filter"
        self.name = name
        # The number of points in our beliefs.
        self.nPoints = nPoints
        # A collection of (x,y,theta) tuples indicating the location and
        # orientation of the known tags.
        self.knownTagLocs = {}

        # The variance of the distance error, as a fraction of the distance
        # between the observer and the tag.
        self.dError = dError
        # The variance of the angle error (radians)
        self.angError = angError
        # The chance that we've misidentified the tag (value in [0,1])
        self.idError = idError
        # A factor by which to increase the dError and andError estimates,
        # due to the possible motion of the camera platform.
        self.motionError = motionError

        # These are meant to keep the id of the currently-observed tag,
        # plus a list of recent observations of that tag.
        self.currentId = -1
        self.currentMsmts = []

        # A counter in which to record the number of empty tag
        # messages in a row.  Even when staring directly at a tag, the
        # ar_recog occasionally throws off an empty Tags message.  So
        # we need a few in a row before being certain there isn't a
        # tag in the field of view.
        self.nEmpty = 0

        # This is the particle filter itself.
        self.guesser = Guesser(name = self.name,
                               periods = (0.0, 0.0, 2*math.pi),
                               data_type = "location",
                               num_bins = (7,7,5))

        # Use this stamp to record the time stamp of a Tags message.
        self.stamp = rospy.Time()

        # When we have something to say (enough measurements to do something
        # with), we'll announce the fact on this publisher.
        self.announce = announce
        self.announced = False

        # This topic is used to edit the list of AR tags in use.
        self.landmarks = landmarks

        # Flag to keep us from processing when we shouldn't.  (Thread safety,
        # as well as data integrity.)
        self.readyToPublish = False
        self.busy = False

        # Flag to indicate whether the robot is in motion.
        self.moving = False

        self.pdebug = rospy.Publisher("/guesser/position/debug", String)

    # First a couple of methods for maintaining the list of known tag locations.
    def handle_lm(self, req):
        """
        Handles a message on the 'landmarks' topic.  Used to modify or query
        the list of AR elements.
        """
        operations = { "remove": self.doRemoveLm,
                       "purge": self.doPurgeLm,
                       "add":    self.doAddLm,
                       "list":   self.doListLm,
                       "nop":    self.doNopLm }
        if req.target and (req.target != self.name):
            return
            
        try:
            operations[req.operation](req)
        except KeyError:
            print "No key called: ", req.operation

        return

    def doPurgeLm(self, req):
        """
        Delete all the known tag locations.
        """
        for idx in self.knownTagLocs.keys():
            self.removeTag(idx)

    def doRemoveLm(self, req):
        for l in req.lm:
            self.removeTag(l.id)

    def doAddLm(self, req):
        for l in req.lm:
            self.addTag(l.id, l.loc)

    def doListLm(self, req):
        out = Lmarks()
        for idx,loc in self.knownTagLocs.items():
            out.lm.append(Lmark(id = idx, 
                                loc = loc))

        out.sender = self.name
        out.operation = "nop"
        if self.landmarks:
            self.landmarks.publish(out)

    def doNopLm(self, req):
        return

    def removeTag(self, idx):
        """
        Removes a tag from the list of known tag locations.
        """
        if idx in self.knownTagLocs.keys():
            del self.knownTagLocs[idx]
            return True
        else:
            return False

    def addTag(self, idx, loc):
        """
        Adds a tag with the given location and idx.  You can also
        use this to move a tag to a new location.
        """
        self.knownTagLocs[idx] = loc

    def setMotionError(self, newError):
        """
        Change the motionError factor on the fly.  Not yet sure this is
        either useful or necessary.
        """
        self.motionError = newError

    def handle_odom(self, odom):
        """
        This method keeps track of whether we're moving.  If we're
        moving, it will affect how and whether to trust a particular
        measurement of a tag.  Also if we're moving we can trim off
        the older elements of the list of measurements, so we're not
        incorporating measurements that are stale.
        """
        if self.busy:
            return

        # If we're not currently tracking anything, then never mind.
        if not self.currentMsmts:
            return

        # Set the moving flag
        if (math.fabs(odom.twist.twist.linear.x) > 0.0001) or \
                (math.fabs(odom.twist.twist.angular.z) > 0.0001):
            self.moving = True
        else:
            self.moving = False

        # Dump any measurement from more than a second ago.
        target = rospy.Time.now().to_sec() - 1.0
        while self.currentMsmts and self.currentMsmts[0].time < target:
            self.currentMsmts.pop(0)

        self.pdebug.publish(String(data="in handle_odom"))

    def handle_image(self, tags):
        """
        Selects the best tag in the image to track.  If it's the same as the
        tag we've already been tracking, add the measurement to the list of
        measurements we're keeping.
        """
        if self.busy:
            return

        tags.header.stamp = tags.header.stamp

        self.pdebug.publish(String(data="in handle_image"))

        self.readyToPublish = False
        # If no tags are in view, get out.
        noneFound = tags.tag_count < 1

        if not noneFound:
            # Select the biggest tag in the image.
            best = Tag(id=-1)
            for tag in tags.tags:
                if (tag.cf > best.cf) and \
                        (tag.id in self.knownTagLocs.keys()):
                    best = tag

            if best.id == -1:
                # We haven't seen a tag we recognize,
                noneFound = True

        else:
            # A few empty messages in a row means we should clear the decks.
            self.nEmpty += 1
            if self.nEmpty > 5:
                self.currentId = -1
                self.currentMsmts = []
                self.announced = False
                self.readyToPublish = False
            return

        # We have at least one tag, so there are no empty tags in a row.
        self.nEmpty = 0

        # Is this the same tag we've been tracking?
        if best.id == self.currentId:
            # Yes.  Add this measurement to the list.
            self.currentMsmts.append(TagLoc(tags.header.stamp, best))
        else:
            # No.  Trash the list and start again.
            self.currentId = best.id
            self.currentMsmts = [ TagLoc(tags.header.stamp, best) ]

        # If we're here, there is at least one tag in the currentMsmts list
        # and so we're ready to propagate our belief.
        if self.announce and not self.announced:
            self.announce.publish(Announce(sender=self.name))
            self.announced = True

        self.readyToPublish = True


    def findPosition(self, tagLoc, tagMsmt):
        """
        If we measured a tag x (which lives at location loc) and got
        the measurement msmt, this function returns the location of
        the measurer in the same reference frame as the tag location.
        """
        rel_x = tagMsmt.loc[0]
        rel_y = tagMsmt.loc[1]

        # Rotate...
        theta = tagLoc[2] + tagMsmt.loc[2]
        x = (rel_x * math.cos(theta)) - (rel_y * math.sin(theta))
        y = (rel_x * math.sin(theta)) + (rel_y * math.cos(theta))

        # ... and translate
        x += tagLoc[0]
        y += tagLoc[1]

        theta += math.pi # Face the other way
        theta = theta % (2 * math.pi)
        if theta > math.pi:
            theta -= (2 * math.pi)

        return (x, y, theta)

    def estimatePosition(self, tagLoc, tagMsmt, dError, angError):
        """
        Uses findPosition() to produce a point that is the estimated
        position plus a randomly-generated error.
        """
        if dError * tagMsmt.dist <= 0:
            print "************", tagMsmt.str()

        epsilon = np.random.normal(scale=dError * tagMsmt.dist) + \
            math.fabs(tagMsmt.dist)

        phi = math.atan2(tagMsmt.loc[1], tagMsmt.loc[0])

        tagMsmtError = TagLoc(rospy.Time.from_sec(tagMsmt.time),
                              (tagMsmt.loc[0] + math.cos(phi) * epsilon,
                               tagMsmt.loc[1] + math.sin(phi) * epsilon,
                               tagMsmt.loc[2] + np.random.normal(scale=angError)))

        return self.findPosition(tagLoc, tagMsmtError)

    def estimatePositions(self, tagLoc, tagMsmt, dError, angError,
                          nEstimates, idError, discardLocs):
        """
        Uses a bunch of calls to estimatePosition() to come up with a set of
        guessed points.  This includes id error (the chance that the given
        tag id is not correct).
        """
        # DiscardLocs is a list of all the tags we know about that are *not*
        # the identified tag.
        out = []
        for i in range(nEstimates):
            idrand = np.random.random()
            if (not discardLocs) or (idrand < idError):
                loc = tagLoc
            else:
                # Pick a random location from discardLocs
                loc = discardLocs[ int((idrand - idError) / (1.0 - idError) * \
                                           len(discardLocs)) ]
            out.append( self.estimatePosition( loc, tagMsmt, dError, angError))

        return out


    def handle_guess(self, greq):
        """
        Takes an input set of points, representing a guess of our position,
        and filters it with a distribution made up from the tag observations
        recorded in the currentMsmts.
        """
        if self.busy or not self.readyToPublish or self.currentId == -1:
            return GuessResponse(no_data=True)
        else:
            self.busy = True

        # Check input types
        assert isinstance(greq, GuessRequest)
        assert isinstance(greq.inPoints, list)
        assert isinstance(greq.inPoints[0], Pt)

        # Make up a distribution of points from the collection of
        # AR observations.
        arPts = Belief()

        arPts.source_stamp = rospy.Time.from_sec(self.currentMsmts[-1].time)
        if self.moving:
            estNPerMsmt = 0
        else:
            estNPerMsmt = max(1, self.nPoints) / len(self.currentMsmts)

        discardLocs = []
        for k in self.knownTagLocs.keys():
            if k != self.currentId:
                discardLocs.append(self.knownTagLocs[k])

        totPoints = self.nPoints
        source_data = "\n"
        for msmt in reversed(self.currentMsmts):
            # print msmt.str()
            if estNPerMsmt == 0:
                nPerMsmt = int(totPoints * 0.4)
                totPoints -= nPerMsmt
            else:
                nPerMsmt = estNPerMsmt

            eps = self.estimatePositions(self.knownTagLocs[self.currentId],
                                         msmt, self.dError, self.angError,
                                         nPerMsmt, self.idError,
                                         discardLocs )
            for ep in eps:
                arPts.points.append(Pt(ep))

            source_data += ("artg(%d), %.4f, %.4f, %.4f " % \
                                ((self.currentId,) + \
                                     self.findPosition(self.knownTagLocs[self.currentId], msmt))) \
                                     + ("(%.4f -- %d)\n" % (msmt.time,nPerMsmt))
            source_data += msmt.str()

        gpts = Belief(points = greq.inPoints,
                      data_type = greq.data_type,
                      source_stamp = greq.source_stamp,
                      source_data = greq.source_data,
                      pers = greq.pers)

        # This is really just here for testing.  See graphic test at EOF
        self.tarPts = arPts

        # n.b. Because of the binning algorithm used to weight the input
        # pointsin the perfesser's particle filter, it makes a
        # difference whether you use distribution A to filter
        # distribution B or the other way around.  You can reverse it
        # by commenting/uncommenting the commented lines and their
        # companions in the section just below here.  Execute this
        # file with an argument to see the effect.

        # Filter it with the input points
#        self.guesser.newPoints(greq.inPoints)
        self.guesser.newPoints(arPts.points)
        mns = self.guesser.means()
        stds = self.guesser.stds()

#        self.guesser.update(arPts)
        self.guesser.update(gpts)

        source_stamp = rospy.Time.from_sec(self.currentMsmts[-1].time)
        source_data += "input: mns, %.3f, %.3f, %.3f," % mns
        source_data += " stds, %.3f, %.3f, %.3f\n" % stds
        source_data += str(self.guesser.hist.bins)
        source_data += "\n"

        # Return
        gresp = GuessResponse(sender=self.name,
                              source_stamp=source_stamp,
                              source_data=source_data,
                              outPoints=self.guesser.outPoints(),
                              no_data=False)

        self.pdebug.publish(String(data="in handle_guess"))
        self.busy = False
        self.announced = False
        return gresp


if __name__ == '__main__':

    rospy.init_node("ar_filter")
    import sys
    import unittest
    class TestARGuesser(unittest.TestCase):
        def setUp(self):
            self.ar = ARGuesser("ar_filter", 1000, 0.1, 0.02, .95, 0.1)

            self.ar.addTag(1, (1.0, 1.0, -1.552))
            self.ar.addTag(2, (1.0, -1.0, 1.552))
            self.ar.addTag(3, (-1.0, -1.0, -1.552))
            self.ar.addTag(4, (-1.0, 1.0, -1.552))

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
            t.xMetric = 1.
            t.yMetric = 1.
            t.zMetric = 0
            ts.tags.append(t)
            ts.tag_count = 1
            ts.header.stamp = rospy.Time.now()

            self.ar.handle_image(ts)

        def testUpdate(self):
            g = Guesser("ar_filter_test", (0.0, 0.0, 2*math.pi),
                        "location", (5,5,1))

            xmax = 2.5 ; ymin = 0.0
            g.uniform(1500, [[0.0,xmax],[ymin,2.0],[3.0,3.25]])

            gr = GuessRequest(inPoints=g.outPoints(),
                              means=g.means(),
                              stds=g.stds(),
                              data_type=g.data_type,
                              pers=g.periods)

            gresp = self.ar.handle_guess(gr)

            mins = map(min, self.ar.guesser.pointArray.transpose())
            maxs = map(max, self.ar.guesser.pointArray.transpose())

            self.assertTrue(mins[1] > ymin)
            self.assertTrue(maxs[0] < xmax)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestARGuesser)
    unittest.TextTestRunner(verbosity=3).run(suite)

    class TestARGuessTwo(unittest.TestCase):
        def setUp(self):
            # Test established 9/22 because filtering seems not to be
            # done correctly in some circumstances.

            self.ar = ARGuesser("ar_filter", 200, 0.5, 0.15, 0.95, 3.0)

            # Add landmark tags
            self.ar.addTag(0, (32.0, 34.85, 1.5708))
            self.ar.addTag(3, (35.0, 39.0, 3.1415))
            self.ar.addTag(9, (29.5, 39.0, 0.0))

            self.tagObs1 = []
            self.tagObs2 = []
            # Add tag observations
            # First set (tag id=9)
            tb = Tags(tag_count = 1)
            tb.tags.append(Tag(distance= 1.099, id= 9, diameter= 78.833,
                               xMetric= 1.076, yMetric=0.222, zMetric= -0.055,
                               xRot= -0.201, yRot=-0.695, zRot=0.013, cf=0.8))
            self.tagObs1.append(tb)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)

            tc = Tags(tag_count = 1)
            tc.tags.append(Tag(distance= 1.152, id= 9, diameter= 75.245,
                               xMetric= 1.128, yMetric=0.233, zMetric= -0.060,
                               xRot= -0.202, yRot=-0.723, zRot=0.017, cf=0.8))
            self.tagObs1.append(tc)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            td = Tags(tag_count = 1)
            td.tags.append(Tag(distance= 1.143, id= 9, diameter= 75.799,
                               xMetric= 1.122, yMetric=0.219, zMetric= -0.061,
                               xRot= -0.200, yRot=-0.701, zRot=0.010, cf=0.8))
            self.tagObs1.append(td)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)

            te = Tags(tag_count = 1)
            te.tags.append(Tag(distance= 1.190, id= 9, diameter= 72.847,
                               xMetric= 1.167, yMetric=0.230, zMetric= -0.066,
                               xRot= -0.201, yRot=-0.721, zRot=0.015, cf=0.8))
            self.tagObs1.append(te)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            tf = Tags(tag_count = 1)
            tf.tags.append(Tag(distance= 1.168, id= 9, diameter= 74.213,
                               xMetric= 1.148, yMetric=0.216, zMetric= -0.063,
                               xRot= -0.191, yRot=-0.692, zRot=0.013, cf=0.8))
            self.tagObs1.append(tf)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            tg = Tags(tag_count = 1)
            tg.tags.append(Tag(distance= 1.217, id= 9, diameter= 71.265,
                               xMetric= 1.196, yMetric=0.227, zMetric= -0.068,
                               xRot= -0.208, yRot=-0.719, zRot=0.017, cf=0.8))
            self.tagObs1.append(tg)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            th = Tags(tag_count = 1)
            th.tags.append(Tag(distance= 1.187, id= 9, diameter= 73.044,
                               xMetric= 1.169, yMetric=0.208, zMetric= -0.066,
                               xRot= -0.207, yRot=-0.696, zRot=0.014, cf=0.8))
            self.tagObs1.append(th)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            ti = Tags(tag_count = 1)
            ti.tags.append(Tag(distance= 1.257, id= 9, diameter= 69.012,
                               xMetric= 1.237, yMetric=0.223, zMetric= -0.072,
                               xRot= -0.208, yRot=-0.716, zRot=0.020, cf=0.8))
            self.tagObs1.append(ti)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            tj = Tags(tag_count = 1)
            tj.tags.append(Tag(distance= 1.227, id= 9, diameter= 70.684,
                               xMetric= 1.210, yMetric=0.202, zMetric= -0.068,
                               xRot= -0.205, yRot=-0.717, zRot=0.017, cf=0.8))
            self.tagObs1.append(tj)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            tk = Tags(tag_count = 1)
            tk.tags.append(Tag(distance= 1.295, id= 9, diameter= 67.001,
                               xMetric= 1.277, yMetric=0.216, zMetric= -0.074,
                               xRot= -0.205, yRot=-0.725, zRot=0.017, cf=0.8))
            self.tagObs1.append(tk)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            tl = Tags(tag_count = 1)
            tl.tags.append(Tag(distance= 1.344, id= 9, diameter= 64.587,
                               xMetric= 1.328, yMetric=0.205, zMetric= -0.080,
                               xRot= -0.212, yRot=-0.718, zRot=0.021, cf=0.8))
            self.tagObs1.append(tl)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)


            tm = Tags(tag_count = 1)
            tm.tags.append(Tag(distance= 1.331, id= 9, diameter= 65.227,
                               xMetric= 1.315, yMetric=0.205, zMetric= -0.079,
                               xRot= -0.209, yRot=-0.722, zRot=0.016, cf=0.8))
            self.tagObs1.append(tm)
            self.tagObs1[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)



            # Second set (tag id=0)

            tn = Tags(tag_count = 1)
            tn.tags.append(Tag(id=0, distance= 4.7, diameter= 20.932,
                               xMetric= 4.027, yMetric=-1.063, zMetric= -0.293,
                               xRot= -0.092, yRot=0.550, zRot=-0.049, cf=0.8))
            self.tagObs2.append(tn)
            self.tagObs2[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)
            #
            to = Tags(tag_count = 1)
            to.tags.append(Tag(id=0, distance= 4.7, diameter= 20.437,
                               xMetric= 4.125, yMetric=-1.089, zMetric= -0.300,
                               xRot= -0.079, yRot=0.466, zRot=-0.045, cf=0.8))
            self.tagObs2.append(to)
            self.tagObs2[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)
            #
            tp = Tags(tag_count = 1)
            tp.tags.append(Tag(id=0, distance= 4.7, diameter= 20.313,
                               xMetric= 4.150, yMetric=-1.096, zMetric= -0.302,
                               xRot= -0.077, yRot=0.441, zRot=-0.041, cf=0.8))
            self.tagObs2.append(tp)
            self.tagObs2[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)
            #
            tq = Tags(tag_count = 1)
            tq.tags.append(Tag(id=0, distance= 4.7, diameter= 19.737,
                               xMetric= 4.400, yMetric=-0.401, zMetric= -0.336,
                               xRot= -0.060, yRot=0.490, zRot=-0.050, cf=0.8))
            self.tagObs2.append(tq)
            self.tagObs2[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)
            #
            tr = Tags(tag_count = 1)
            tr.tags.append(Tag(id=0, distance= 4.7, diameter= 18.840,
                               xMetric= 4.627, yMetric=-0.135, zMetric= -0.362,
                               xRot= 0.016, yRot=-0.089, zRot=-0.003, cf=0.8))
            self.tagObs2.append(tr)
            self.tagObs2[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)
            #
            ts = Tags(tag_count = 1)
            ts.tags.append(Tag(id=0, distance= 4.7, diameter= 18.803,
                               xMetric= 4.636, yMetric=-0.135, zMetric= -0.363,
                               xRot= 0.002, yRot=-0.013, zRot=-0.005, cf=0.8))
            self.tagObs2.append(ts)
            self.tagObs2[-1].header.stamp = rospy.Time.now()+rospy.Duration(2.5)

            # Create a list of points
            self.odompts = Belief()
            self.odompts.source_stamp = rospy.Time.now()
            self.odompts.data_type = "position"
            self.odompts.pers = (0.0,0.0,2*math.pi)

            self.odompts.points.append(Pt(point = [30.270074844360352, 37.276561737060547, -1.0008981227874756]))
            self.odompts.points.append(Pt(point = [31.114320755004883, 39.706657409667969, -1.5473997592926025]))
            self.odompts.points.append(Pt(point = [30.135814666748047, 38.370811462402344, -1.888274073600769]))
            self.odompts.points.append(Pt(point = [29.945672988891602, 38.871379852294922, -1.9299575090408325]))
            self.odompts.points.append(Pt(point = [30.730808258056641, 39.201541900634766, -1.3144114017486572]))
            self.odompts.points.append(Pt(point = [29.748968124389648, 40.282516479492188, -1.3278772830963135]))
            self.odompts.points.append(Pt(point = [30.07977294921875, 39.100208282470703, 0.13855478167533875]))
            self.odompts.points.append(Pt(point = [30.89521598815918, 39.265548706054688, -0.091095283627510071]))
            self.odompts.points.append(Pt(point = [31.879093170166016, 38.302749633789062, -1.2434898614883423]))
            self.odompts.points.append(Pt(point = [30.101011276245117, 37.854793548583984, -2.2228705883026123]))
            self.odompts.points.append(Pt(point = [30.709014892578125, 37.843536376953125, 0.043588083237409592]))
            self.odompts.points.append(Pt(point = [30.506467819213867, 40.010456085205078, -1.4747337102890015]))
            self.odompts.points.append(Pt(point = [31.59398078918457, 38.53076171875, -0.33048191666603088]))
            self.odompts.points.append(Pt(point = [30.303289413452148, 39.495014190673828, -1.1399352550506592]))
            self.odompts.points.append(Pt(point = [29.33857536315918, 39.171646118164062, 0.4430752694606781]))
            self.odompts.points.append(Pt(point = [31.683307647705078, 39.878280639648438, 0.45741814374923706]))
            self.odompts.points.append(Pt(point = [31.454641342163086, 38.439685821533203, -0.88907504081726074]))
            self.odompts.points.append(Pt(point = [31.779428482055664, 40.044418334960938, -1.0684206485748291]))
            self.odompts.points.append(Pt(point = [33.568550109863281, 37.695934295654297, -2.0175380706787109]))
            self.odompts.points.append(Pt(point = [31.545112609863281, 40.480720520019531, -0.54006654024124146]))
            self.odompts.points.append(Pt(point = [30.170516967773438, 38.498809814453125, -1.4701484441757202]))
            self.odompts.points.append(Pt(point = [30.409576416015625, 37.683376312255859, -1.6718564033508301]))
            self.odompts.points.append(Pt(point = [29.52964973449707, 37.850334167480469, -0.63223099708557129]))
            self.odompts.points.append(Pt(point = [29.667976379394531, 37.973346710205078, -0.26350811123847961]))
            self.odompts.points.append(Pt(point = [29.563600540161133, 39.335124969482422, -1.232595682144165]))
            self.odompts.points.append(Pt(point = [29.958351135253906, 39.422828674316406, -1.4827215671539307]))
            self.odompts.points.append(Pt(point = [30.907115936279297, 38.345264434814453, -0.86614459753036499]))
            self.odompts.points.append(Pt(point = [30.131557464599609, 40.022842407226562, -1.4664720296859741]))
            self.odompts.points.append(Pt(point = [29.411039352416992, 38.902793884277344, -1.0057836771011353]))
            self.odompts.points.append(Pt(point = [30.85606575012207, 39.832427978515625, 2.1577854156494141]))
            self.odompts.points.append(Pt(point = [29.731876373291016, 38.825115203857422, -3.2315242290496826]))
            self.odompts.points.append(Pt(point = [31.087825775146484, 38.975418090820312, -0.95082145929336548]))
            self.odompts.points.append(Pt(point = [29.840570449829102, 36.879241943359375, -1.4251528978347778]))
            self.odompts.points.append(Pt(point = [30.260429382324219, 37.692165374755859, -2.4271883964538574]))
            self.odompts.points.append(Pt(point = [30.780357360839844, 37.517978668212891, -0.22615325450897217]))
            self.odompts.points.append(Pt(point = [31.090944290161133, 39.49420166015625, -1.2875828742980957]))
            self.odompts.points.append(Pt(point = [29.951889038085938, 37.681797027587891, -1.8173259496688843]))
            self.odompts.points.append(Pt(point = [30.736438751220703, 38.453472137451172, 1.581925630569458]))
            self.odompts.points.append(Pt(point = [31.131669998168945, 40.106861114501953, -0.83733350038528442]))
            self.odompts.points.append(Pt(point = [31.267986297607422, 38.791103363037109, -1.6219151020050049]))
            self.odompts.points.append(Pt(point = [30.381004333496094, 37.916427612304688, -2.9471356868743896]))
            self.odompts.points.append(Pt(point = [31.262250900268555, 38.32220458984375, 2.6640346050262451]))
            self.odompts.points.append(Pt(point = [31.711343765258789, 39.202610015869141, 0.50152063369750977]))
            self.odompts.points.append(Pt(point = [29.093973159790039, 39.068305969238281, 2.1263768672943115]))
            self.odompts.points.append(Pt(point = [31.947713851928711, 39.396633148193359, -1.2701097726821899]))
            self.odompts.points.append(Pt(point = [32.701225280761719, 38.701332092285156, 0.24179227650165558]))
            self.odompts.points.append(Pt(point = [31.235294342041016, 38.062767028808594, 0.16368959844112396]))
            self.odompts.points.append(Pt(point = [30.621139526367188, 38.841678619384766, 0.5664251446723938]))
            self.odompts.points.append(Pt(point = [31.228311538696289, 38.973983764648438, -1.3841959238052368]))
            self.odompts.points.append(Pt(point = [31.227518081665039, 37.836814880371094, -0.20462854206562042]))
            self.odompts.points.append(Pt(point = [28.787836074829102, 37.290367126464844, -1.1039118766784668]))
            self.odompts.points.append(Pt(point = [30.223196029663086, 38.303703308105469, 0.12392342090606689]))
            self.odompts.points.append(Pt(point = [30.266559600830078, 40.117767333984375, 0.048228900879621506]))
            self.odompts.points.append(Pt(point = [32.680503845214844, 38.587146759033203, 2.2451503276824951]))
            self.odompts.points.append(Pt(point = [30.964853286743164, 39.185844421386719, 0.68032699823379517]))
            self.odompts.points.append(Pt(point = [31.371484756469727, 39.056087493896484, -1.2882604598999023]))
            self.odompts.points.append(Pt(point = [31.613655090332031, 38.621128082275391, -1.8427245616912842]))
            self.odompts.points.append(Pt(point = [29.429080963134766, 39.133907318115234, 0.32223188877105713]))
            self.odompts.points.append(Pt(point = [31.014055252075195, 39.036296844482422, -2.1791789531707764]))
            self.odompts.points.append(Pt(point = [30.411550521850586, 38.491737365722656, -0.26150518655776978]))
            self.odompts.points.append(Pt(point = [30.956239700317383, 38.966907501220703, 0.7661360502243042]))
            self.odompts.points.append(Pt(point = [30.870798110961914, 39.434917449951172, 0.95150887966156006]))
            self.odompts.points.append(Pt(point = [30.593788146972656, 38.584999084472656, -1.7893637418746948]))
            self.odompts.points.append(Pt(point = [30.464975357055664, 39.130100250244141, 1.3042105436325073]))
            self.odompts.points.append(Pt(point = [30.099998474121094, 39.793872833251953, 0.80829918384552002]))
            self.odompts.points.append(Pt(point = [31.663578033447266, 39.002937316894531, -1.0626286268234253]))
            self.odompts.points.append(Pt(point = [29.842536926269531, 39.075618743896484, 1.5789744853973389]))
            self.odompts.points.append(Pt(point = [29.976461410522461, 36.587936401367188, 1.6872138977050781]))
            self.odompts.points.append(Pt(point = [31.462844848632812, 38.017406463623047, -1.1369251012802124]))
            self.odompts.points.append(Pt(point = [30.490106582641602, 38.500354766845703, 0.48916232585906982]))
            self.odompts.points.append(Pt(point = [29.391929626464844, 38.099872589111328, 2.5543980598449707]))
            self.odompts.points.append(Pt(point = [29.247026443481445, 39.624031066894531, -0.12120623141527176]))
            self.odompts.points.append(Pt(point = [31.503973007202148, 38.446205139160156, -3.1286356449127197]))
            self.odompts.points.append(Pt(point = [31.653970718383789, 38.667770385742188, -0.47884169220924377]))
            self.odompts.points.append(Pt(point = [30.52598762512207, 39.030082702636719, -0.43365344405174255]))
            self.odompts.points.append(Pt(point = [30.081937789916992, 39.037113189697266, 2.1071567535400391]))
            self.odompts.points.append(Pt(point = [31.562776565551758, 37.681240081787109, 0.066303707659244537]))
            self.odompts.points.append(Pt(point = [31.038400650024414, 38.53607177734375, -0.0039447052404284477]))
            self.odompts.points.append(Pt(point = [29.373147964477539, 38.920730590820312, 2.1989076137542725]))
            self.odompts.points.append(Pt(point = [29.511135101318359, 38.785400390625, -1.1610405445098877]))
            self.odompts.points.append(Pt(point = [31.148481369018555, 40.048736572265625, -0.45902588963508606]))
            self.odompts.points.append(Pt(point = [30.239738464355469, 37.850273132324219, 0.28272825479507446]))
            self.odompts.points.append(Pt(point = [31.337991714477539, 39.672714233398438, -0.76640599966049194]))
            self.odompts.points.append(Pt(point = [30.787784576416016, 38.842262268066406, 3.0053062438964844]))
            self.odompts.points.append(Pt(point = [30.474359512329102, 38.81695556640625, -0.50923788547515869]))
            self.odompts.points.append(Pt(point = [30.439788818359375, 39.451328277587891, -0.97366964817047119]))
            self.odompts.points.append(Pt(point = [30.562446594238281, 39.272079467773438, -2.2275233268737793]))
            self.odompts.points.append(Pt(point = [30.087718963623047, 37.775371551513672, -1.6903327703475952]))
            self.odompts.points.append(Pt(point = [30.892177581787109, 38.983356475830078, -0.77668571472167969]))
            self.odompts.points.append(Pt(point = [30.991443634033203, 38.398059844970703, -1.8947417736053467]))
            self.odompts.points.append(Pt(point = [31.975458145141602, 39.230796813964844, 0.26103943586349487]))
            self.odompts.points.append(Pt(point = [32.498130798339844, 38.959556579589844, -0.24292692542076111]))
            self.odompts.points.append(Pt(point = [31.950014114379883, 39.813278198242188, -0.36810517311096191]))
            self.odompts.points.append(Pt(point = [29.617813110351562, 39.581241607666016, 0.36815297603607178]))
            self.odompts.points.append(Pt(point = [30.450927734375, 38.242229461669922, 0.28132376074790955]))
            self.odompts.points.append(Pt(point = [30.383977890014648, 37.963085174560547, -1.610014796257019]))
            self.odompts.points.append(Pt(point = [30.997392654418945, 38.653293609619141, -0.91360276937484741]))
            self.odompts.points.append(Pt(point = [30.767295837402344, 39.558174133300781, 0.68181848526000977]))
            self.odompts.points.append(Pt(point = [31.776302337646484, 38.9053955078125, -0.37009438872337341]))
            self.odompts.points.append(Pt(point = [30.411872863769531, 38.323375701904297, -0.28608328104019165]))
            self.odompts.points.append(Pt(point = [30.626457214355469, 39.616054534912109, 0.54964369535446167]))
            self.odompts.points.append(Pt(point = [29.86622428894043, 40.649749755859375, -0.16955955326557159]))
            self.odompts.points.append(Pt(point = [30.514865875244141, 38.233043670654297, 0.39115497469902039]))
            self.odompts.points.append(Pt(point = [28.940126419067383, 39.146995544433594, -0.81445831060409546]))
            self.odompts.points.append(Pt(point = [29.24431037902832, 38.138015747070312, -0.7187422513961792]))
            self.odompts.points.append(Pt(point = [29.646816253662109, 38.456089019775391, -0.74024856090545654]))
            self.odompts.points.append(Pt(point = [29.705410003662109, 37.907421112060547, -1.9927434921264648]))
            self.odompts.points.append(Pt(point = [28.759370803833008, 38.611934661865234, 1.3372275829315186]))
            self.odompts.points.append(Pt(point = [30.252048492431641, 37.833740234375, 0.48776131868362427]))
            self.odompts.points.append(Pt(point = [29.022079467773438, 38.782295227050781, 0.88095295429229736]))
            self.odompts.points.append(Pt(point = [29.023496627807617, 37.668254852294922, -0.45949405431747437]))
            self.odompts.points.append(Pt(point = [29.545303344726562, 38.702442169189453, 0.65847760438919067]))
            self.odompts.points.append(Pt(point = [29.540021896362305, 38.693744659423828, -0.055521693080663681]))
            self.odompts.points.append(Pt(point = [29.053129196166992, 38.425331115722656, -0.18365171551704407]))
            self.odompts.points.append(Pt(point = [30.515298843383789, 39.368236541748047, -1.9954359531402588]))
            self.odompts.points.append(Pt(point = [32.518085479736328, 39.211875915527344, -2.3149206638336182]))
            self.odompts.points.append(Pt(point = [32.021389007568359, 38.758434295654297, -0.68849021196365356]))
            self.odompts.points.append(Pt(point = [30.598861694335938, 38.264316558837891, -0.89071094989776611]))
            self.odompts.points.append(Pt(point = [30.87043571472168, 37.997066497802734, 0.92923122644424438]))
            self.odompts.points.append(Pt(point = [32.27423095703125, 40.59661865234375, -0.8519139289855957]))
            self.odompts.points.append(Pt(point = [30.757022857666016, 38.55328369140625, -0.9695279598236084]))
            self.odompts.points.append(Pt(point = [30.670272827148438, 38.767917633056641, 0.24692307412624359]))
            self.odompts.points.append(Pt(point = [29.162986755371094, 38.274017333984375, 0.96348696947097778]))
            self.odompts.points.append(Pt(point = [30.016912460327148, 38.718021392822266, 0.61936092376708984]))
            self.odompts.points.append(Pt(point = [28.155014038085938, 40.419826507568359, -0.971718430519104]))
            self.odompts.points.append(Pt(point = [28.871601104736328, 39.362140655517578, 1.1135703325271606]))
            self.odompts.points.append(Pt(point = [29.992000579833984, 38.778961181640625, 0.46872898936271667]))
            self.odompts.points.append(Pt(point = [28.182270050048828, 39.057991027832031, -0.8176039457321167]))
            self.odompts.points.append(Pt(point = [30.536195755004883, 37.199016571044922, 0.51089465618133545]))
            self.odompts.points.append(Pt(point = [30.667434692382812, 38.446773529052734, -1.0376116037368774]))
            self.odompts.points.append(Pt(point = [30.829135894775391, 38.08367919921875, -2.4810590744018555]))
            self.odompts.points.append(Pt(point = [31.26807975769043, 38.400177001953125, -0.95246243476867676]))
            self.odompts.points.append(Pt(point = [30.42823600769043, 39.672401428222656, 0.056227635592222214]))
            self.odompts.points.append(Pt(point = [29.670656204223633, 38.658206939697266, 0.29575285315513611]))
            self.odompts.points.append(Pt(point = [30.695398330688477, 39.071125030517578, -0.031552296131849289]))
            self.odompts.points.append(Pt(point = [30.351222991943359, 38.880474090576172, -0.34638351202011108]))
            self.odompts.points.append(Pt(point = [30.772645950317383, 38.272670745849609, 0.75682216882705688]))
            self.odompts.points.append(Pt(point = [31.139507293701172, 37.83575439453125, 0.54553472995758057]))
            self.odompts.points.append(Pt(point = [30.991975784301758, 38.860881805419922, -0.089838244020938873]))
            self.odompts.points.append(Pt(point = [30.224460601806641, 39.090724945068359, -0.75413179397583008]))
            self.odompts.points.append(Pt(point = [29.545866012573242, 37.646148681640625, 0.14901883900165558]))
            self.odompts.points.append(Pt(point = [30.971744537353516, 39.297237396240234, 1.379436731338501]))
            self.odompts.points.append(Pt(point = [30.732986450195312, 39.691425323486328, -0.10063297301530838]))
            self.odompts.points.append(Pt(point = [32.044101715087891, 38.140151977539062, 0.3567022979259491]))
            self.odompts.points.append(Pt(point = [30.698894500732422, 38.206855773925781, 2.3262193202972412]))
            self.odompts.points.append(Pt(point = [30.474531173706055, 39.135837554931641, -2.3984625339508057]))
            self.odompts.points.append(Pt(point = [31.564060211181641, 37.858577728271484, -0.03700970858335495]))
            self.odompts.points.append(Pt(point = [29.701057434082031, 38.574546813964844, 0.4018617570400238]))
            self.odompts.points.append(Pt(point = [31.562524795532227, 39.078746795654297, -0.32565015554428101]))
            self.odompts.points.append(Pt(point = [30.155597686767578, 37.116161346435547, -1.0973734855651855]))
            self.odompts.points.append(Pt(point = [29.064529418945312, 37.994350433349609, 0.24882923066616058]))
            self.odompts.points.append(Pt(point = [30.55891227722168, 38.256759643554688, -0.86716204881668091]))
            self.odompts.points.append(Pt(point = [30.278509140014648, 39.697257995605469, 1.2950208187103271]))
            self.odompts.points.append(Pt(point = [30.582019805908203, 39.405479431152344, -1.7986055612564087]))
            self.odompts.points.append(Pt(point = [31.754474639892578, 38.329532623291016, -0.30225017666816711]))
            self.odompts.points.append(Pt(point = [31.163017272949219, 38.82110595703125, 0.29308557510375977]))
            self.odompts.points.append(Pt(point = [30.549434661865234, 39.493759155273438, -0.89030647277832031]))
            self.odompts.points.append(Pt(point = [30.739089965820312, 38.057415008544922, -0.26652032136917114]))
            self.odompts.points.append(Pt(point = [29.600696563720703, 39.145545959472656, -0.3522757887840271]))
            self.odompts.points.append(Pt(point = [31.241081237792969, 37.804714202880859, -1.0003501176834106]))
            self.odompts.points.append(Pt(point = [30.481830596923828, 38.964286804199219, 0.28248691558837891]))
            self.odompts.points.append(Pt(point = [31.935159683227539, 37.793506622314453, -1.5654968023300171]))
            self.odompts.points.append(Pt(point = [29.937103271484375, 37.765132904052734, -0.5121954083442688]))
            self.odompts.points.append(Pt(point = [28.643606185913086, 38.730056762695312, 0.073277167975902557]))
            self.odompts.points.append(Pt(point = [28.103324890136719, 37.804328918457031, -1.2989633083343506]))
            self.odompts.points.append(Pt(point = [29.354150772094727, 39.644420623779297, 0.10506143420934677]))
            self.odompts.points.append(Pt(point = [29.138891220092773, 38.832275390625, -2.1455531120300293]))
            self.odompts.points.append(Pt(point = [29.969367980957031, 40.036235809326172, 1.1739670038223267]))
            self.odompts.points.append(Pt(point = [30.853143692016602, 37.959754943847656, 0.12812867760658264]))
            self.odompts.points.append(Pt(point = [29.75050163269043, 39.174552917480469, 0.66152924299240112]))
            self.odompts.points.append(Pt(point = [29.874223709106445, 39.261405944824219, 1.1186970472335815]))
            self.odompts.points.append(Pt(point = [31.505485534667969, 39.621452331542969, 1.0868011713027954]))
            self.odompts.points.append(Pt(point = [29.344432830810547, 39.698024749755859, 0.85172152519226074]))
            self.odompts.points.append(Pt(point = [30.172971725463867, 39.589546203613281, -0.11735149472951889]))
            self.odompts.points.append(Pt(point = [29.608112335205078, 38.228668212890625, 0.29214423894882202]))
            self.odompts.points.append(Pt(point = [29.903663635253906, 40.956867218017578, 1.7403619289398193]))
            self.odompts.points.append(Pt(point = [30.399726867675781, 39.447090148925781, 0.60924994945526123]))
            self.odompts.points.append(Pt(point = [29.383575439453125, 39.712833404541016, -0.73392635583877563]))
            self.odompts.points.append(Pt(point = [29.613353729248047, 39.688446044921875, 0.64189529418945312]))
            self.odompts.points.append(Pt(point = [30.239702224731445, 39.745269775390625, -2.3885297775268555]))
            self.odompts.points.append(Pt(point = [30.063028335571289, 40.498588562011719, 1.2968816757202148]))
            self.odompts.points.append(Pt(point = [30.022090911865234, 37.514808654785156, -1.1431752443313599]))
            self.odompts.points.append(Pt(point = [29.944574356079102, 38.708213806152344, -1.4335638284683228]))
            self.odompts.points.append(Pt(point = [29.405088424682617, 38.66583251953125, 0.28575631976127625]))
            self.odompts.points.append(Pt(point = [30.308006286621094, 38.505802154541016, -0.48474344611167908]))
            self.odompts.points.append(Pt(point = [29.079109191894531, 38.599273681640625, -0.32485926151275635]))
            self.odompts.points.append(Pt(point = [30.053359985351562, 38.388397216796875, 0.88482797145843506]))
            self.odompts.points.append(Pt(point = [30.33607292175293, 39.580268859863281, 0.051641646772623062]))
            self.odompts.points.append(Pt(point = [30.917247772216797, 38.212799072265625, -1.9683822393417358]))
            self.odompts.points.append(Pt(point = [31.216379165649414, 39.148357391357422, 0.25117239356040955]))
            self.odompts.points.append(Pt(point = [31.037786483764648, 39.095508575439453, -0.87645959854125977]))
            self.odompts.points.append(Pt(point = [31.781658172607422, 39.273998260498047, -0.63625097274780273]))
            self.odompts.points.append(Pt(point = [30.717601776123047, 38.970401763916016, 0.71739351749420166]))
            self.odompts.points.append(Pt(point = [30.477115631103516, 41.163211822509766, 1.2543141841888428]))
            self.odompts.points.append(Pt(point = [30.424484252929688, 38.102809906005859, -1.7516540288925171]))
            self.odompts.points.append(Pt(point = [31.209266662597656, 39.486442565917969, -0.50954741239547729]))
            self.odompts.points.append(Pt(point = [29.378799438476562, 39.024517059326172, 0.17917396128177643]))
            self.odompts.points.append(Pt(point = [30.168901443481445, 40.208621978759766, -0.226235032081604]))
            self.odompts.points.append(Pt(point = [30.234504699707031, 38.637657165527344, -0.3363739550113678]))
            self.odompts.points.append(Pt(point = [30.75672721862793, 39.2183837890625, -0.23922370374202728]))

        def testFilter(self):
            # We're testing the likelihood of one set of points interfering
            # with the subsequent set.  So here's a previous set to try
            # and mess things up with.

            self.ar.guesser.newPoints(self.odompts.points)

            from nav_msgs.msg import Odometry
            odomtest = Odometry()
            odomtest.twist.twist.linear.x = 0.1996
            odomtest.twist.twist.angular.z = -0.4

            for ts in self.tagObs1:
                self.ar.handle_image(ts)
                rospy.sleep(0.1)

            self.ar.handle_odom(odomtest)

            gr = GuessRequest(source_stamp = rospy.Time.now(),
                              inPoints=self.ar.guesser.outPoints())
            gr.means = self.ar.guesser.means()
            gr.stds = self.ar.guesser.stds()
            gr.pers = self.ar.guesser.periods
            gr.data_type = self.ar.guesser.data_type
            gr.source_data = self.ar.guesser.source_data

            gresp = self.ar.handle_guess(gr)

            rospy.sleep(1.01)

            for ts in self.tagObs2:
                self.ar.handle_image(ts)
                rospy.sleep(0.1)

            self.ar.handle_odom(odomtest)
            self.ar.guesser.newPoints(self.odompts.points)
            gr = GuessRequest(source_stamp = rospy.Time.now(),
                              inPoints=self.ar.guesser.outPoints())
            gr.means = self.ar.guesser.means()
            gr.stds = self.ar.guesser.stds()
            gr.pers = self.ar.guesser.periods
            gr.data_type = self.ar.guesser.data_type
            gr.source_data = self.ar.guesser.source_data


            gresp = self.ar.handle_guess(gr)

            for p in gresp.outPoints:
                print p.point

            print gresp.source_data

            print self.ar.guesser.hist.str()

            print self.ar.guesser.means()

            print self.ar.guesser.hist.mode()

            #######  TEST UNFINISHED: NEED ASSERTIONS HERE #######


    suite = unittest.TestLoader().loadTestsFromTestCase(TestARGuessTwo)
    unittest.TextTestRunner(verbosity=3).run(suite)







    if len(sys.argv) > 1:

        ar = ARGuesser("ar_filter", 1000, 0.12, 0.02, .95, 1.0)

        ar.addTag(1, (1.0, 1.0, -1.552))
        ar.addTag(2, (-1.0, 1.0, -1.552))
        ar.addTag(3, (-1.0, -1.0, 1.552))
        ar.addTag(4, (1.0, -1.0, 1.552))

        ts = Tags()
        tp1 = Tag()
        tp2 = Tag()
        tp3 = Tag()
        ts.tag_count = 1

        tp1.id = 3
        tp1.cf = 0.930898971454
        tp1.x = 142
        tp1.y = 119
        tp1.diameter = 71.5479359083
        tp1.distance = 0.595
        tp1.xRot = -0.621133277614
        tp1.yRot = -0.302759286446
        tp1.zRot = 0.000895010569522
        tp1.xMetric = 0.593719991196
        tp1.yMetric = 0.039007074243
        tp1.zMetric = 0.00216395929358
        tp1.cwCorners = [105.17465970602461, 87.180871529283905, 168.5823416618432, 90.684715782579005, 185.36736180920431, 146.57086825994284, 114.83379062405047, 149.98069475970726]


        tp2.id = 3
        tp2.cf = 0.930898971454
        tp2.x = 142
        tp2.y = 120
        tp2.diameter = 67.7943606478
        tp2.distance = 0.630
        tp2.xRot = -0.625798167888
        tp2.yRot = -0.300316644741
        tp2.zRot = 0.00313719601005
        tp2.xMetric = 0.628644696561
        tp2.yMetric = 0.0413018823538
        tp2.zMetric = -0.0
        tp2.cwCorners = [107.786537271992, 89.209214203735826, 167.41109201676878, 92.682617935465444, 183.65128817276795, 145.48275048573961, 116.27770022215505, 148.56224047509554]

        tp3.id = 3
        tp3.cf = 0.930898971454
        tp3.x = 142
        tp3.y = 120
        tp3.diameter = 67.768295348
        tp3.distance = .630
        tp3.xRot = -0.624097966914
        tp3.yRot = -0.299587727591
        tp3.zRot = 0.00247547510095
        tp3.xMetric = 0.628644696561
        tp3.yMetric = 0.0413018823538
        tp3.zMetric = -0.0
        tp3.cwCorners = [107.75202856521415, 89.293641064934647, 167.48568979098783, 92.59522799867139, 183.55603345988703, 145.487104357096, 116.25198744326647, 148.56341574640874]


        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
        ts.tags = [ tp1 ]
        ts.header.stamp = rospy.Time.now()
        ar.handle_image(ts)
        rospy.sleep(0.1)
        ts.tags = [ tp2 ]
        ts.header.stamp = rospy.Time.now()
        ar.handle_image(ts)
        rospy.sleep(0.1)
        ts.tags = [ tp3 ]
        ts.header.stamp = rospy.Time.now()
        ar.handle_image(ts)

        import tgraph
        tg = tgraph.Tgraph(300,220)

        g = Guesser("ar_filter_test", (0.0, 0.0, 2*math.pi),
                    "location", (1,1,1))

        #g.uniform(1000, [[-5.0,5.0],[-5.0,5.0],[-3.0,3.25]])
        g.normal(1000, [[-1.0, 0.5], [0.0, 1.0], [-1.5, 1.0]])

        
        # Draw initial location estimate
        tg.new_graph()
        corners = np.array([[-4.0, -4.0, -4.0], [4.0, 4.0, 4.0]])
        tg.draw_scatter(corners, 0, 1, 2, "s")

        tg.draw_scatter(g.pointArray, 0,1,2, "s", recalc=False)


        print "*****************************************************"

        greq = GuessRequest()
        greq.source_stamp = rospy.Time.now()
  #      greq.source_stamp.secs -= 1
        greq.inPoints = g.outPoints()

        gresp = ar.handle_guess(greq)

        # Draw estimate from AR tag observations
        tg.new_graph()
        parray = np.array([p.point for p in ar.tarPts.points])
        tg.draw_scatter(parray, 0, 1, 2, "s", recalc = False)

        tg.plot_points(np.array([[-1,-1,1.55],[-1,1,1.55],[1,1,-1.55],[1,-1,-1.55]]), 0,1,2, "c")

        # Draw resulting filter result: new position estimate.
        tg.new_graph()
        parray = np.array([p.point for p in gresp.outPoints])
        tg.draw_scatter(parray, 0,1,2, "s", recalc=False)

#        tg.plot_points(ar.guesser.hist.plottable(), 0,1,2,"c")

        tg.plot_points(np.array([[-1,-1,1.55],[-1,1,1.55],[1,1,-1.55],[1,-1,-1.55]]), 0,1,2, "c")



        tg.mainloop()



# Tags.msg
# Header header
# uint32 image_width
# uint32 image_height
# float64 angle_of_view
# uint32 tag_count
# Tag[] tags


# Tag.msg
#    Number from object symbol file
# uint32 id
#    Confidence, 0-1
# float64 cf
#    Image coordinates, pixels
# uint32 x
# uint32 y
#   Real, not as observed
# float64 diameter
#   Distance between camera and target
# uint32 distance
#   Rotation of tag relative to camera?
# float64 xRot
# float64 yRot
# float64 zRot
#   Distance, to tag
# float64 xMetric
# float64 yMetric
# float64 zMetric
#   corner coordinates, pixels?
# float64[8] cwCorners


#    def handle_guess(self, guess):
#        pass

