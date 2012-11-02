#!/usr/bin/env python
#
# This is the controlling logic for a 2-dimensional robot, as in model-2d.
#
import roslib ; roslib.load_manifest('cover')
import rospy
from cover.msg import Room, Room2D
from cover.srv import MRFQuery, MRFQueryRequest, MRFQueryResponse
from perfesser.msg import Belief, Pt
from perfesser_guesser import Guesser
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
from model_phi import Phi
import numpy as np
import math


class Neighbor(object):
    """
    This class stores information about a neighboring robot, including
    the last message it sent to us and its location (presumably
    measured by AR tag or radio signal strength or something).
    """
    def __init__(self, name):
        self.name = name

        self.lsub = rospy.Subscriber(self.name + "/guesser/position/guess", 
                                     Belief, self.getPos, 
                                     queue_size = 1, buff_size = 200)

        # Use this to hold our measurement of the neighboring robot's
        # position.
        self.location = Belief(data_type = "location",
                               data_sub_type = self.name,
                               pers = (0.0,0.0))
        # Use this to hold the last question sent to the neighboring
        # robot.  This 'question' would have been a suggestion about
        # appropriate targets for that neighboring robot to move
        # towards.
        self.lastQuestion = Belief(data_type = "destination",
                                   data_sub_type = self.name,
                                   pers = (0.0,0.0))
        # This is the reply to the question stored above, a collection
        # of ratings of each of the points in the question above.
        # Each point is a floating-point value x: 0 < x < 1.
        self.lastReply = Belief(data_type = "rating",
                                data_sub_type = "from " + self.name,
                                pers = (0.0,0.0))

        # This is the service by which we send the lastQuestion and
        # receive the lastReply.
        self.advise = rospy.ServiceProxy(self.name + "/advice",
                                         MRFQuery)

        # This is the latest suggestion sent to us by this neighbor.
        self.lastSuggestion = Belief(data_type = "suggestion",
                                   data_sub_type = self.name,
                                   pers = (0.0,0.0))
                   
        # The reply we made to the suggestion above.
        self.rating = Belief(data_type = "rating",
                             data_sub_type = "from host",
                             pers = (0.0,0.0))


    def str(self):
        out = ""
        out += "Name=%s, Location=%.3f\n" % (self.name,) + self.pos
        out += str(self.location)
        out += str(self.lastQuestion)
#        out += str(self.lastReply)
        return out

    def getPos(self, req):
        """
        Receives a Belief object containing the robot's location
        estimate.
        """
        self.location = req


class coverControl(object):
    def __init__(self, name, neighborDist = 2.0, npoints = 70):
        print "Starting." , name
    
        self.name = name
        self.npoints = npoints
        self.neighborDist = neighborDist

        # This is a list of the robots on our team, assumed to contain
        # Neighbor() objects, as above.
        self.neighbors = []
        # This is a list of the robots within neighborDist of us.
        self.neighborNames = []

        # This is an array of points in configuration space indicating
        # where we need to go next.
        self.deltaConfig = np.array([[]])

        self.twist = Twist()

        self.maxSpeed = 0.5 # 0.25
        self.maxTurn = 1.0 # .6

        self.kSpeed = 15.0
        self.kTurn = 13.0

        # These are coefficients for the opinion formation.  Tune and forget.
        self.aw = 0.5
        self.sp = 0.2 # Without this term, you get an exploding ring more often.
        self.av = 0.6
        self.se = 0.0

        self.maxDist = 2.5

        self.phi = Phi(self.name, 
                       self.aw, self.sp, self.av, self.se, 
                       self.maxDist)

        self.movePeriod = 5.0
        self.nextMoveTime = rospy.Time.now().to_sec() + self.movePeriod


        # Estimated variance in the position measurements.
        self.std = 0.01

        # When rating a neighbor's opinion about our desired position,
        # use this many bins.  Shouldn't be a radically different
        # order of magnitude from npoints.
        self.nbins = (5,5)

        rospy.sleep(1.0)

        self.odomSub = rospy.Subscriber(self.name + "/odom", Odometry, 
                                        self.trackOdom,
                                        queue_size = 1, buff_size = 250)

        self.advice = rospy.Service(self.name + "/advice", MRFQuery, 
                                    self.takeAdvice)

        self.busy = False

    def addNeighbor(self, name):
        self.neighbors.append(Neighbor(name))

                             
    def addNeighbors(self, names):
        for name in names:
            self.addNeighbor(name)


    def recordNeighbors(self):
        """
        At any time, the self.neighbors list should contain an entry for 
        all the robots in the experiment and the self.nbrNames list should
        contain the names of the robots whose positions are close enough to 
        consider true neighbors.
        """
        self.neighborNames = []
        for nb in self.neighbors:
            if self.dist(self.pos, nb.location.means) < self.neighborDist:
                self.neighborNames.append(nb.name)

        if not self.neighborNames:
            # No neighbors -- should act on that.  For now just
            # return.
            return


    def trackBump(self, req):
        self.phi.addObstacle(req.means)


    def trackPos(self, req):
        """
        This gets called as often as a new position estimate is issued.
        """
        self.posBelief = req
        self.pos = req.means

        self.phi.updateOccupancy(self.pos)

        now = rospy.Time.now().to_sec()
        if self.nextMoveTime - now < self.movePeriod:
            self.evaluateMove()
            self.nextMoveTime = now + self.movePeriod


    def takeAdvice(self, req):
        """
        Takes a piece of advice from its neighbor, rates it, and
        returns the rating.  The advice is a distribution of points to
        which the other robot thinks this robot ought to go.  We
        compare that advice to our own intentions and return ratings
        indicating the degree of agreement between the two: 1.0 is
        complete agreement and 0.0 is complete disagreement.  We never
        return exactly 0.0 as a matter of policy.
        """
        if not req.question:
            return MRFQueryResponse(no_data = True)
        if not self.deltaConfig.any():
            return MRFQueryResponse(no_data = True)

        # Allocate our intention into bins.
        question = self.ptsToArray(req.question)

        # Establish bins so that there is one point in each outer bin.
        # (The Histogram class in perfesser is set up so the outer
        # bins will be empty, which is why we're not using it here.
        mins = [ min(self.deltaConfig[:,0]), min(self.deltaConfig[:,1]) ]
        maxs = [ max(self.deltaConfig[:,0]), max(self.deltaConfig[:,1]) ]

        bins = [ np.linspace(mn + 1.e-10, mx - 1.e-10, bn) for \
                     mn,mx,bin in zip(mins, maxs, self.nbins) ]
        hist = np.zeros(self.nbins)

        # Create a distribution of points that includes neighbor's
        # opinions except for the one asking the question.
        phi = self.deltaConfig[:,(0,1)]
        
        messageProduct = [ 1.0 ] * phi.shape[0]
        for nb in self.neighbors:
            if (nb.name != req.asker) and (nb.lastReply.points):
                messageProduct = [ m * p.point[0] for m,p in \
                                   zip(messageProduct,
                                       nb.lastReply.points) ]

        phi = self.resample(phi, messageProduct)

        # Populate bins
        for delta in phi:
            hist[sum(delta[0] > bins[0])][sum(delta[1] > bins[1])] += 1.0

        n = sum(sum(hist))

        # Formulate a reply by binning the input data.
        m = MRFQueryResponse(no_data = False,
                             source_stamp = req.source_stamp)
        for q in req.question:
            m.reply.append(Pt(point=( \
                        hist[sum(q.point[0] > bins[0])][sum(q.point[1] > bins[1])]/n, )))

        # print "entering service: ", self.name
        # print self.name, "'s opinion: ", self.deltaConfig
        # print "question: ",question
        # print "hist: ", hist
        # print "bins: ", bins
        # print m
        return m

    def ptsToArray(self, pts):
        return np.array([p.point for p in pts])

    def arrayToPts(self, array):
        return [ Pt(point = a) for a in array ]

    def setAdvice(self, name, advice):
        """
        Takes a name of a robot and an array of advice and stores it
        in the neighbor array -- if appropriate.
        """
        for nb in self.neighbors:
            if nb.name == name:
                nb.lastQuestion.points = self.arrayToPts(advice)

    def evaluateMove(self):
        """
        Formulates a distribution of points representing a belief
        about the direction in which we should move, along with our
        neighbors.  The belief is based on our measurements of our own
        location, as well as the location of the other robots, along
        with some goals (i.e. spread out, explore, etc).

        To be explicit, inputs are these:
          - Location of self
          - Location of neighbors
          - Suggestions from neighbors about where self should move. 
          - Replies by neighbors to suggestions about where they
        should move.

        The first two form the input to the phi() function.  The third
        is the input to the psi() function, and the fourth is the
        messageProduct referenced below.

        Definition: "configuration space" is the n-dimensional space
        where each point represents a particular configuration of all
        the robots in it.  So a one-dimensional room containing three
        robots corresponds to a three-dimensional configuration
        space.  A two-dimensional room with five robots makes ten
        dimensions in configuration space.  Seems confusing, but it
        makes some of the math easier.
        """
        # Marshal the inputs: our position and the positions of our
        # neighbors.  Park them all in an array where each row
        # represents a position in configuration space and the
        # collection of rows represent the distribution of those
        # points.
        currentConfig = np.array(self.ptsToArray(self.posBelief.points))[:,(0,1)]

        # For 2d this should be two entries with X and Y
        self.deltaNames = [ self.name + "/dx", self.name + "/dy" ] 

 
        # All the robots are in the neighbors list, but only the nearby ones
        # are to be used.
        for nb in self.neighbors:
            if nb.location.points:
                currentConfig = np.hstack([currentConfig, 
                                           self.ptsToArray(nb.location.points)])

                self.deltaNames += [ nb.name + "/dx", nb.name + "/dy" ]


        # Run through the points in the current configuration and
        # generate predictions for each point in the distribution.
        deltaPts = []
        for cpoint in currentConfig:
            deltaPts.append(self.phi.phi(cpoint))

        self.deltaConfig = np.array(deltaPts)


        # Break up the array, and store the opinions about each
        # neighbor in the neighbor list.  We do it this way in case
        # the neighbor list has changed since the marshaling.
        for nm,dc in zip(self.deltaNames,np.hsplit(self.deltaConfig,
                                         self.deltaConfig.shape[1])):
            self.setAdvice(nm, dc)

        # Notify each neighbor of our opinion (and record their
        # responses).
        messageProduct = [ 1.0 ] * self.npoints
        for nb in self.neighbors:

            # If we don't already have an opinion about where this
            # robot should be then pass for the moment and we'll get
            # it the next time around.
            if not nb.lastQuestion.points:
                break

            m = MRFQueryRequest(source_stamp = rospy.Time.now(),
                                asker = self.name,
                                asked = nb.name,
                                question = nb.lastQuestion.points)
            r = nb.advise(m)
            if r.reply:
                nb.lastReply.points = r.reply
                nb.lastReply.source_stamp = r.source_stamp
                messageProduct = [ lr.point[0] * mp \
                                       for lr, mp in zip(r.reply,
                                                         messageProduct) ]

        # Resample based on the neighbor's responses.
        desiredConfig = self.resample(self.deltaConfig, messageProduct)

        # The first set of coordinates in this is the desired place
        # for us to move.
        moves = desiredConfig[:,(0,1)]

        self.goThatWay(moves)

        self.busy = False

        
    def goThatWay(self, moves):
        """
        Takes an array of points to move to and selects a direction in
        which to move in order to get us there.
        """
        self.goal = (self.pos[0] + np.mean(moves[:, 0]), 
                     self.pos[1] + np.mean(moves[:, 1]), 0.0)

        #######  SET NEW GOAL HERE #######



    def weightedSample(self, pointProbs, numSamples):
        """
        The following sampling algorithm is adapted from a sneaky
        algorithm for coming up with a selection of N random numbers
        between 0 and Q.  The idea is that the probability that all N
        of the numbers are less than some X is P=(X/Q)**N.  Solve for
        X in terms of P, and you get an equation that will pick the
        largest number in that distribution for some random P.  Do
        that N times, and you get N samples.
    
        The input is an indexed array of probabilities (use
        enumerate(P) for the input, e.g. and the number of samples to
        take from that array.  The output is an iterator that will
        provide the indices you can use to sample the original array
        with, as in:
    
        output = locArray[weightedSample(enumerate(locProbs), n)

        This won't exactly work, since the enumerate has to be turned
        into a sequence before it can be subscripted (as below) and
        the same is true of the function output.

        For more: stackoverflow.com/questions/2140787
        """

        total = sum(prob for i, prob in pointProbs)
        assert(not math.isnan(total))
        j = 0
        i, prob = pointProbs[j]
        while numSamples:
            x = total * (1 - np.random.random() ** (1.0/numSamples))
            total -= x
            while x > prob:
                x -= prob
                j += 1
                i, prob = pointProbs[j]
            prob -= x
            yield i
            numSamples -= 1

    def resample(self, array, ratings):
        """
        Resamples the distribution represented by the samples in
        array, based on the list of ratings.  There should be one
        rating for each row of the array, and each rating should be
        greater than zero and less than or equal to 1.0.  The ratings
        list need not be normed.
        """
        #print "resample according to: ", ratings

        newseq = self.weightedSample([(i,p) for i,p in enumerate(ratings)], 
                                     len(ratings))
        #print "comes up with: ", [p for p in newseq]

        wiggle = lambda x: self.std * (np.random.random() - 0.5) + x

        out = []
 
        for pt in array[ [ i for i in newseq ], :]:
            out.append([ wiggle(pt[0]), wiggle(pt[1]) ])

        return np.array(out)



