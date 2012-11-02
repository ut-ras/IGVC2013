#!/usr/bin/env python
#
# The Perfesser is a set of guessing routines, meant to assimilate
# guesses on some topic from a wide variety of sources.  We are
# catholic in our tastes, and don't particularly care what the data
# we're guessing about represent.  All we care about is that the data
# be represented by a tuple of floating-point numbers, one for each
# dimension of the problem space.  Periodic dimensions are permitted.
#
# The guesses are delivered to us via a ROS topic in a message format
# specified in the msg/ subdirectory, per ROS standards.
#
# The process here is that an instance of the perfesser keeps a
# collection of data points around.  Periodically, the data points in
# the collection are "filtered" by assimilating a set of new data
# points.  The resulting distribution should be more or less the
# intersection of the set of points kept internally here and some set
# of points developed by the sensor filter, resampled to keep the same
# number of data points.
#
# For a first pass at implementation, the filtration is implemented by
# turning the collection of current points into a histogram and using
# that to toss out the outlying points among the input data, and to
# weight the remaining points.  We then resample the collection to
# restore the original number of points, and to create a collection
# that are of equal probability.
#
# Note that the odometry filter has a somewhat privileged position
# here.  It operates by taking the perfesser's points and updating
# them according to its understanding of where the robot has traveled
# since the perfesser's points were developed.  In the lingo of the
# Bayesian analysis at the root of all this, the odom_filter is
# advancing the priors, and the other filters represent the
# posteriors.  The odom_filter does the p(z(t)|z(t-i)) part of the job
# while the others do the p(x(t)|z(t))
#
#
#
# The name?  The Perfesser Guesser was a remarkable performer at the
# Barnstable County Fair in the 1970s and 1980s.  (Probably at a lot
# of other fairs, too, but that's where I saw him.)  For 25c he'd
# guess your name, height, weight, shoe size, year of birth, make of
# car, favorite color, whatever.  If he guessed right, he'd keep your
# quarter, and if he was wrong, you'd get a little prize worth no more
# than 15c.  His delivery was flawless -- a boisterous combination of
# overweening confidence and insult -- you only had to listen for a
# moment or two before you felt a deep need to shut him up by stumping
# him.  So you'd try, and probably win and he'd make a dime and you'd
# get a prize, and the satisfaction of shutting him up temporarily.
# The result: everyone was happy.
#
# Tom Sgouros 05/2011
#
import roslib ; roslib.load_manifest('perfesser')
import rospy
import math
import random
import numpy as np
from perfesser.msg import Pt, Belief

class Histogram(object):
    """
    Use this to create and store a multi-dimensional histogram: a
    collection of spatial bins and the counts of the points that fall
    in those bins.  A histogram created with one collection of points
    will be used to filter a second collection.

    NOTE ABOUT PERIODIC DIMENSIONS: This class knows nothing about
    periodic dimensions.  The binning is done strictly according to
    the < and > operations.  If you use this with a set of points, some
    of whose dimensions are periodic, you have to treat the discontinuity
    in your code, either by binning with a tangent, or adjusting the
    values so the mean is somewhere away from the discontinuity.
    """

    def __init__(self, points, nbins):
        """
        Inputs:

        points An m x n array, where n is the number of dimensions
              we're looking at and m is the number of points.  If you
              want to ignore one of the columns of the array, set it
              to have one bin.  This is a numpy-style array.

        nbin The number of bins to use in each dimension.  This tuple
              must have as many entries as there are columns in the
              points array.  Too few will give an error.  Too many
              doesn't fail, but gives bad results.  Note that the
              algorithm tries to create empty bins to bound the
              array.  This means that specifying two bins in a
              dimension will give funny results.  Use one bin to
              ignore a dimension (take all values) but otherwise use
              three or more.

        Data:
       
        bins  A list of bin boundaries for each dimension.  A
              three-dimensional data set will have three lists of bin
              boundaries.

        hist  A multi-dimensional array containing the count of data
              points that appear in a given bin.

        """
        self.numBins = nbins
        self.rebin(points, nbins)

        # Norming is not necessary, but this is how you would do it.
        # self.hist = self.hist/self.hist.sum()

    def get(self, point):
        """
        Get() returns the value of the histogram at the tuple given by point.
        """
        assert(self.full)
        return self.hist[tuple(map(lambda pt,bn: sum(pt > bn), point, self.bins))]

    def all(self):
        """
        Creates an empty histogram with three bins for each dimension.
        """
        self.bins = [ [-1.e37 , 1.e37] for n in self.numBins ]
        self.numBins = [3] * len(self.numBins)

    def populate(self, points):
        """
        Populate the bins with a set of data (list of Pt objects).
        Doesn't move the bin boundaries.
        """
        if self.full:
            self.hist = np.zeros(self.numBins)

        for point in points:
            self.hist[ \
                tuple(map(lambda pt,bn: sum(pt > bn), point, self.bins))] += 1
        self.full = True


    def rebin(self, points, numBins):
        """
        Define the bin boundaries, and fill them.
        Inputs: a list of Pt objects, and a tuple with the number of bins to 
        use in each dimension.
        """
        self.numBins = numBins

        # Calculate maxes and mins based on the input array.
        self.mins = [ row.min() for row in points.transpose() ]
        self.maxs = [ row.max() for row in points.transpose() ]

        # Calculate the bin boundaries.  Notice that we cheat a tiny bit so
        # that the data minimum is safely inside a bin.  This class is agnostic
        # about periodic dimensions.  You can treat them properly either by
        # making sure there are enough bins, or by adjusting the values so the
        # means are distant from the range discontinuity.
        self.bins = [ np.linspace(mn - math.fabs(mn*1e-7),mx,(n - 1)) for \
            mn,mx,n in zip(self.mins, self.maxs, self.numBins) ]

        # Initialize the histogram array
        self.hist = np.zeros(self.numBins)

        # Flag to indicate whether the bins have data.  These don't yet.
        self.full = False

        self.populate(points)


    def md(self, h):
        """
        Returns the value and location of the input array's maximum.
        We do this recursively, by calling md() on each element of h.  If 
        h is a 2-d array, we call it on each row, and if it's a 1-d array, 
        we call it on each element.  The h.shape test differentiates np array
        elements that are iterable.
        """
        if h.shape:
            out = []
            for el in h:
                out.append(self.md(el))
            m = max(enumerate(out),key=lambda x:x[1])
            return (m[1][0], [m[0]] + m[1][1])
        else:
            return (h, [])

    def mode(self):
        """
        Returns the x and y value corresponding to the middle of the
        most highly occupied bin.  In the event that the most highly occupied
        bin is on the border, we estimate the value by extrapolating the
        bin width.  Note that this can be highly misleading for some cases.
        """
        assert(self.full)
        m = self.md(self.hist)
        out = []
        try:
            for i,b in zip(m[1], self.bins):
                if i == len(b):
                    out.append( b[i-1] + (b[1] - b[0])/2.0 )
                elif i == 0:
                    out.append( b[0] - (b[1] - b[0])/2.0 )
                else:
                    out.append( (b[i] + b[i-1])/2.0 )

            return tuple(out)

        except IndexError:
            print "IndexError in mode()"
            print "m = ", m
            print "bins", self.bins
            print self.hist
            return
        except TypeError:
            print "TypeError in mode()"
            print "m = ", m
            print "bins", self.bins
            print self.hist
            return

    def binmeans(self):
        """
        Returns some numbers we can use to characterize the bins.  There is
        a mean-substitute and a variance-substitute for each dimension.  The
        mean substitute is just the mean of the bin boundaries.  The variance
        substitute is the width of the bins.
        """
        meansub = [] ; varsub = []
        for i in range(len(self.bins)):
            meansub.append(np.mean(self.bins[i]))
            varsub.append(self.bins[i][1] - self.bins[i][0])
        return (meansub, varsub)

    def salt(self, nPerBin, bins):
        """
        Provides a collection of random points, uniformly distributed in
        each of the occupied bins of the histogram.  For dimensions with
        only one bin, we use a normal distribution around the mean.  This
        will break on dimensions with fewer than one bin.

        Inputs: 

        nPerBin -- The number of random points to put in each bin.

        bins -- the collection of bin boundaries, as in self.bins.

        This was implemented but never used in the localization stack,
        but has been left in because who knows what uses this may have
        in the future.
        """
        if len(bins) > 1:
            outl = self.salt(nPerBin * (len(bins[0]) - 1), bins[1:])
            out = []
            for o in outl:
                out.append([bins[0][0] + (bins[0][-1] - bins[0][0]) * \
                                random.random()] + o)
        else:
            out = []
            for i in range(nPerBin * (len(bins[0]) - 1)):
                out.append([bins[0][0] + (bins[0][-1] - bins[0][0]) * \
                                random.random()])
        return out

    def str(self):
        """
        Returns a printable representation of the histogram
        """
        out = ""
        out += "{0}\n".format(str(self.hist))
        for i in range(len(self.bins)):
            out += "{0}\n".format(str(self.bins[i]))
        return out

    def plot_recurse(self, pos, vals):
        """
        This is a tool used in the plottable() method.  It takes a
        list of lists like this: ((a,b,c),(d,e,f)) and produces this:
        ((a,d),(a,e),(a,f),(b,d),(b,e),(b,f),(c,d),(c,e),(c,f)).
        """
        out = []
        for p in pos:
            pl = p if isinstance(p,list) else [p]
            for x in vals[0]:
                out.append(pl + [x])
        if vals[1:]:
            return self.plot_recurse(out, vals[1:])
        else:
            return out


    def plottable(self):
        """
        Returns an array suitable for plotting with the scatter
        routine in tgraph.  Each point is in the middle of a bin and a
        column is added to hold the count from that bin.

        Note that for high-dimensional space, you may also want to
        fold some of the bins into one another.  That is, if you have
        bins set up for theta, and are plotting x and y, you'll get a
        bunch of values for each (x,y) point, corresponding to the
        various values for the theta bins.
        """
        binMids = []
        for i in range(len(self.bins)):
            if self.numBins[i] > 1:
                step = self.bins[i][1] - self.bins[i][0]
                a = self.bins[i] - step/2.0
                a = np.append(a,  a[-1] + step)
            else:
                a = np.append(self.bins[i], (self.maxs[i] + self.mins[i])/2.0)
            binMids.append(a)

        # There is probably a better and more Python-esque way to do this
        # next step, but we use my klugy recursion instead.
        pts = self.plot_recurse(binMids[0], binMids[1:])

        return np.array(pts, dtype = np.float32)

class Guesser(object):
    """
    Maintains an array of multi-dimensional points representing the
    current best guess of something multi-dimensional.  Dimensions can
    be periodic, indicated with the periods array.  The assumption is
    that all the points in the array are equally probable solutions to
    something.

    The guess can be updated with new information, delivered in the
    form of another collection of possible data points.  The original
    guess is filtered with this new guess, and the resulting points
    are resampled so they once again represent equal probabilities.

    The guess is maintained as a numpy-style array, but the input and
    output is done with Belief objects, per the ROS message defined as
    part of the perfesser package.

    PERIODIC DIMENSIONS: Periodic dimensions are confined to a single
    span of the period, hopefully centered at something approximating
    the dimension's mean.  At first, we assume the mean to be zero, so
    incoming points are confined to the range (-p/2, p/2), but later,
    when it becomes clear there is a non-zero mean, incoming points
    are confined to (m - p/2, m + p/2) where m is the mean.  We
    attempt to keep the means within the range (-p, p), more or less.
    """
    def __init__(self, name, periods, data_type, num_bins, 
                 data_sub_type="", dim_names = False, pub=False):
        """
        Inputs:

        name  The name of this node.

        periods An array of n values, one for each dimension of the
              data points being manipulated here.  For non-periodic
              dimensions, use zero, and for periodic dimensions, use
              the period.

        data_type A string to indicate the kind of data we're dealing
              with, e.g. "location", "temperature", whatever.

        data_sub_type A string to further modify the data type under
              consideration, so two nodes could both be considering
              location, but one might be the location of robot abel
              and the other of robot baker.

        dim_names A list of strings, a name for each of the dimensions. 

        num_bins A list to feed to the Histogram class, to define the
              number of bins in each dimension of the histogram used
              to generate the particle filter weights.

        pub   A publisher that publishes estimates of the quantity the
              Perfesser is tracking.  The topic puts out Belief
              objects.
        """

        self.periods = periods
        self.numBins = num_bins
        self.name = name
        self.data_type = data_type
        self.data_sub_type = data_sub_type
        if dim_names:
            self.dim_names = dim_names
        else:
            self.dim_names = [ "" ] * len(self.periods)
        self.pub = pub

        # This variable keeps track of the appropriate range for
        # comparing periodic variables.  That is, if a variable has a
        # mean at 0.49 * p, then it's better to use (0,p) as your
        # variable limits than (-p/2, p/2).  Specifically what this
        # variable contains is the lower limit of the desired range.
        self.periodRanges = [ -p/2.0 for p in periods ]

        # This is the current guess, a numpy-style array of
        # hypothetical points.  Empty, just to hold the place.
        self.nPoints = 10
        self.pointArray = np.zeros((self.nPoints,
                                    len(self.periods)),
                                   dtype=float)

        # This is used to record the time stamps of the most recent data
        # to go into the current state of the pointArray.  This data should
        # have been delivered with the updated data from each filter.
        self.stamp = rospy.Time()

        # This is just a string forwarded to the perfesser by each of
        # its filters.  Use it to indicate how the most recent guess
        # was developed.
        self.source_data = ""

        # This is just an optimization for the stds calculator.  After
        # calculating the means, we store them here for use later.
        self.meansList = [ 0.0 ] * len(self.periods)

        # Initialize the histogram.
        self.makeHist()
        self.hist.all()

    def meanCal(self, pts, period):
        """
        Special mean calculator.  Calculates the mean if the period is
        zero, and a reasonable mean substitute for a periodic x.  Periodic
        dimensions are returned in the (-p/2, p/2) range
        """
        if (period == 0):
            return np.mean(pts)
        else:
            pfactor = period / (2 * math.pi)
            c = []
            s = []
            for th in pts:
                c.append( math.cos(th * pfactor) )
                s.append( math.sin(th * pfactor) )
            return pfactor * math.atan2(np.mean(s), np.mean(c))

    def delta(self, x, y, period):
        """
        The difference between two bearings is the shortest way around
        the circle from one to the other.
        """
        provisionalDelta = x - y
        if period:
            provisionalDelta = provisionalDelta % period
            if (math.fabs(provisionalDelta) > period/2.0):
                return provisionalDelta - math.copysign(period, provisionalDelta)
            else:
                return provisionalDelta
        else:
            return provisionalDelta

    def cartesianDistance(self, pt1, pt2):
        """
        Calculates a cartesian distance between two points.  Periodic
        dimensions are treated like other dimensions, except that the distance
        between two periodic values is defined to be the shorter way around
        the circle.
        """
        dist = 0.0
        for c1,c2,per in zip(pt1, pt2, self.periods):
            dist += self.delta(c1, c2, per)**2
        return dist**0.5

    def stdCal(self, pts, period, mean):
        """ 
        Special variance calculator for periodic variables. 
        """
        if (period == 0):
            return np.std(pts)
        else:
            deltas = [ self.delta(x, mean, period)**2 for x in pts ]
            return (sum(deltas)/len(pts))**0.5

    def means(self):
        """
        Returns the centroid of the maintained data points.
        """
        self.meansList = map(self.meanCal,
                             self.pointArray.transpose(),
                             self.periods)

        return tuple(self.meansList)

    def stds(self):
        """
        Returns the variances of the distribution.  Call this right
        after calling means().
        """
        return tuple( map( self.stdCal,
                           self.pointArray.transpose(),
                           self.periods,
                           self.meansList))

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
     
        See also: stackoverflow.com/questions/2140787
        """
        total = sum(prob for i, prob in pointProbs)
        assert(not math.isnan(total))
        j = 0
        i, prob = pointProbs[j]
        while numSamples:
            x = total * (1 - random.random() ** (1.0/numSamples))
            total -= x
            while x > prob:
                x -= prob
                j += 1
                i, prob = pointProbs[j]
            prob -= x
            yield i
            numSamples -= 1

    def pointsToArray(self, inBelief):
        """
        Takes an input Belief object and unpacks it into an
        numpy-style array.
        """
        assert isinstance(inBelief, Belief)

        outArray = np.array([self.fixPerPoint(p.point) for p in inBelief.points])

        return outArray

    def arrayToBelief(self, inArray):
        """
        Takes the numpy-style array in self.pointArray and packs it
        into an otherwise empty Belief object.  The object is
        returned, but should usually have its secondary data filled in
        before being published or used anywhere.
        """
        assert isinstance(inArray, np.ndarray)

        return Belief( points = [ Pt(point=row) for row in inArray ] )

    def outPoints(self):
        """
        Returns the point array as a list of Pt objects.
        """
        return [ Pt(point = row) for row in self.pointArray ]

    def belief(self):
        """
        Returns the belief embodied by the current state of the Guesser.
        """
        return Belief( source_stamp = self.stamp,
                       source_data = self.source_data,
                       data_type = self.data_type,
                       data_sub_type = self.data_sub_type,
                       sender = self.name,
                       means = self.means(),
                       stds = self.stds(),
                       pers = self.periods,
                       dim_names = self.dim_names,
                       points = self.outPoints())


    def fixPerPoint(self, inPoint):
        """
        Guarantees that the periodic dimensions are within the
        acceptable ranges, as chosen by choosePeriodRange() below.
        """
        assert(len(inPoint) == len(self.periods))

        outPoint = []
        for inval,per,minval in zip(inPoint, self.periods, self.periodRanges):
            halfp = per/2.0
            maxval = minval + per
            if per:
                outval = inval % per
                outval = outval - per if outval >= maxval else outval
                outPoint.append(outval)
            else:
                outPoint.append(inval)

        return outPoint


    def newPoints(self, newPoints):
        """
        Replaces the array of estimated points with a new array,
        provided as a list of Point objects.
        """
        assert isinstance(newPoints, list)
        assert isinstance(newPoints[0], Pt)

        self.nPoints = len(newPoints)

        # This seems like it could use optimization.
        # Read in the array
        self.pointArray = np.array([ p.point for p in newPoints ])

        # Choose the right range for the periodic dimensions.
        self.choosePeriodRanges()

        # Read in the array again, fixing the periodic columns as you go.
        self.pointArray = np.array([ self.fixPerPoint(p.point) \
                                         for p in newPoints ])

    def choosePeriodRanges(self):
        """
        Sets the acceptable range of the periodic dimensions.  We try
        to keep the means of the data values as far from the
        discontinuity as possible.  So if the mean of a 2pi-periodic
        value is 1, the acceptable range is -2.14 to 4.14.
        """
        self.periodRanges = [ (m - (p/2.0) if p else 0.0) \
                                  for m,p in zip(self.means(), self.periods)]

    def makeHist(self):
        """
        Generate a histogram for the points in the pointArray (the current
        guess).
        """
        self.hist = Histogram(self.pointArray, self.numBins)

    def update(self, newBelief):
        """
        Transforms the current guess into a histogram and uses it to
        filter the incoming points.  The resulting array is resampled
        to get us back to the same number of points as we began with.
        Uses the numBins list passed to the guesser initializer.

        The input is a Belief object, which is a collection of Point
        objects, each representing a guess.  Internally, we use numpy
        style arrays, so the first and last step is conversion.
        """
        self.makeHist()

        # Record the time stamp of the incoming data.
        self.stamp = newBelief.source_stamp
        self.source_data = newBelief.source_data

        # Create a prediction of the probability of each input point
        # straight from the histogram
        inputArray = self.pointsToArray(newBelief)
        newPointPs = [ self.hist.get(newPt) for newPt in inputArray ]
        self.nPoints = len(newPointPs)

        if sum(newPointPs):
            newPointPs = [ (i,p) for i,p in enumerate(newPointPs) ]
        else:
            # If you're here, the probability distributions don't overlap.
            # Create a prediction of probability based on the distance from
            # the current distribution's mean.  This is something of a hack.
            means = self.means()
            newPointPs = []
            for newPt in inputArray:
                newPointPs.append(self.cartesianDistance(newPt, means))

            mn = min(newPointPs) * 0.9
            mx = max(newPointPs)
            newPointPs = [ (i,(((mx-mn)/(p-mn))**2.0)-1.0) \
                               for i,p in enumerate(newPointPs) ]
            self.source_data += "DISJOINT\n"

        # Now sample the input points based on that probability.
        newSeq = [ i for i in self.weightedSample(newPointPs, self.nPoints) ]

        # Assign the result to the points that make up the current guess...
        self.pointArray = inputArray[ newSeq ]

        # Publish if that's what's called for...
        self.publish()

        # ...and get out.

    def publish(self):
        """
        This is only published here during debugging and model use.  If the
        perfesser.py is in use, the output of the perfesser is published
        there.
        """
        if self.pub:
            p = self.arrayToBelief(self.pointArray)
            p.means = self.means()
            p.stds = self.stds()
            p.pers = self.periods
            p.data_type = self.data_type
            p.data_sub_type = self.data_sub_type
            p.sender = self.name
            p.header.stamp = rospy.Time.now()
            self.pub.publish(p)


    def normal(self, nPoints, ranges):
        """
        Creates a set of normally-distributed data points with the
        center and standard deviations given in the ranges argument.
        The ranges argument is a set of pairs, with a center and std
        for each dimension.  Use this to initialize the perfesser in
        certain cases.
        """
        assert(len(ranges) == len(self.periods))

        self.nPoints = nPoints
        self.pointArray = np.zeros((self.nPoints,
                                    len(self.periods)),
                                   dtype=float)

        for i in range(self.nPoints):
            point = []
            for c,period in zip(ranges, self.periods):
                cmpt = np.random.normal(loc=c[0],scale=c[1])
                if period:
                    cmpt = cmpt % period
                    cmpt = cmpt if cmpt < period/2.0 else cmpt - period
                point.append(cmpt)
            self.pointArray[i] = np.array(point)


    def uniform(self, nPoints, ranges):
        """
        Creates a set of uniformly-distributed data points within the
        ranges specified.  The input argument is a set of pairs, with
        a min and max for each dimension.
        """
        assert(len(ranges) == len(self.periods))

        self.nPoints = nPoints
        points = []

        spans = [ rmax - rmin for rmin,rmax in ranges ]
        for i in range(self.nPoints):
            points.append( \
                [ r[0] + np.random.random() * span \
                      for r,span in zip(ranges, spans) ])

        self.pointArray = np.array(points)

if __name__ == '__main__':


    import unittest
    class TestHistogram(unittest.TestCase):
        def setUp(self):
            data = []
            periods = (0.0, 0.0, 2*math.pi, 0.0, 5.0)
            self.n = 1000.0
            for i in range(int(self.n)):
                ran = [np.random.random() for i in range(len(periods))]
                d = [ ((r*p) - (p / 2.0)) if p else r for r,p in zip(ran, periods) ]
                data.append(d)

            points = np.array(data)

            self.h = Histogram(points, (4,4,4,4,4))

        def testRecurse(self):
            a = [[1,2],[3,4],[5,6]]
            b = self.h.plot_recurse(a[0],a[1:])

            testList = [[1, 3, 5], 
                        [1, 3, 6], 
                        [1, 4, 5], 
                        [1, 4, 6], 
                        [2, 3, 5], 
                        [2, 3, 6], 
                        [2, 4, 5], 
                        [2, 4, 6]]
            self.assertEqual(b, testList)

        def testBins(self):
            # The sum over the whole histogram should equal the number of
            # data points.
            self.assertEqual(self.h.hist.sum(), self.n)
            # The sum where any dimension is zero or 3 should be zero.
            self.assertEqual(self.h.hist[2][3].sum(), 0.0)
            self.assertEqual(self.h.hist[0].sum(), 0.0)
            # The sum where no dimension is zero or 3 should be positive.
            self.assertTrue(self.h.hist[2][2].sum() > 0.0)

            # These are statistical tests.  Shouldn't fail, but weird things
            # happen.
            self.assertTrue(self.h.bins[0][0] < 0.005)
            self.assertTrue(math.fabs(0.5 - self.h.bins[0][1]) < 0.01)
            self.assertTrue(math.fabs(1.0 - self.h.bins[0][2]) < 0.01)

        def testMode(self):
            m = self.h.md(self.h.hist)

            self.assertEqual(m[0], self.h.get(self.h.mode()))

    print "TESTING Histogram"
    suite = unittest.TestLoader().loadTestsFromTestCase(TestHistogram)
    unittest.TextTestRunner(verbosity=3).run(suite)

    from perfesser_tester import TestPublisher

    class TestGuesser(unittest.TestCase):
        def setUp(self):
            self.pub = TestPublisher()
            self.g = Guesser("TestGuesser", (0.0,0.0,2*math.pi),
                             "position", (4,4,7), pub=self.pub)
            self.xloc = 1.0
            self.yscale = 0.5
            self.thloc = 3.0
            self.thscale = 1.0
            self.g.normal(1000, [[self.xloc,1.0],[0.0,self.yscale],
                           [self.thloc, self.thscale]])

        def testObject(self):
            self.assertTrue(self.pub.testObject)

        def testMeans(self):
            m = self.g.means()
            s = self.g.stds()

            ntrials = 75 ; thres = 0.08
            xlocflag = False ; yscaleflag = False
            thlocflag = False ; thscaleflag = False
            for i in range(ntrials):
                xlocflag = xlocflag or (math.fabs(m[0] - self.xloc) < thres)
                yscaleflag = yscaleflag or (math.fabs(s[1] - self.yscale) < thres)
                thlocflag = thlocflag or (math.fabs(m[2] - self.thloc) < thres)
                thscaleflag = thscaleflag or (math.fabs(s[2] - self.thscale) < thres)
                self.setUp()

            self.assertTrue(xlocflag)
            self.assertTrue(yscaleflag)
            self.assertTrue(thlocflag)
            self.assertTrue(thscaleflag)

        def testConversion(self):
            nentries = 30 ; ndims = 3
            inputArray = np.zeros((nentries, ndims), dtype=float)
            for i in range(nentries):
                for j in range(ndims):
                    inputArray[i][j] = np.random.random()

            p = self.g.arrayToBelief(inputArray)

            p.sender = "hello"
            outputArray = self.g.pointsToArray(p)

            self.assertEqual(inputArray[0][0], outputArray[0][0])
            self.assertEqual(inputArray[nentries-1][ndims-1],
                             outputArray[nentries-1][ndims-1])

        def testUpdate(self):
            # Create a uniform distribution
            rospy.init_node('perfesser')

            pts = Belief()
            for i in range(1000):
                x = np.random.random()
                y = np.random.random()
                th = np.random.random()

                p = Pt()
                p.point = [x, y, th]
                pts.points.append(p)

            xmxflag = False ; ymxflag = False ; thmxflag = False
            xmnflag = False ; ymnflag = False ; thmnflag = False

            ntrials = 5 ; thres = 0.01
            for i in range(ntrials):
                self.g.update(pts)

                mx = map(max, self.g.pointArray.transpose())
                mn = map(min, self.g.pointArray.transpose())

                xmxflag = xmxflag or ((math.fabs(mx[0]) - 1.0) < thres)
                ymxflag = ymxflag or ((math.fabs(mx[0]) - 1.0) < thres)
                thmxflag = thmxflag or ((math.fabs(mx[0]) - 1.0) < thres)
                xmnflag = xmnflag or (math.fabs(mn[0]) < thres)
                ymnflag = ymnflag or (math.fabs(mn[0]) < thres)
                thmnflag = thmnflag or (math.fabs(mn[0]) < thres)
                self.setUp()

            self.assertTrue(self.pub.testObject)

            self.assertTrue(xmxflag)
            self.assertTrue(ymxflag)
            self.assertTrue(thmxflag)
            self.assertTrue(xmnflag)
            self.assertTrue(ymnflag)
            self.assertTrue(thmnflag)

    print "TESTING Guesser -- ***** Please make sure roscore is running! *****"
    suite = unittest.TestLoader().loadTestsFromTestCase(TestGuesser)
    unittest.TextTestRunner(verbosity=3).run(suite)


    import sys
    if len(sys.argv) > 1:

        g = Guesser("graphGuesser", (0.0,0.0,2*math.pi),
                    "position", (4,4,7))
        g.normal(1500, [[1.0,1.0],[0.0,0.3],[-0.3,0.3]])
        #g.uniform([[0.5,2.0],[-1.0,0.75],[0.5,1.5]])

        means = g.means()

        import tgraph

        tg = tgraph.Tgraph(400,350)

        corners = np.array([[-4.5, -4.5,-4.0],
                            [7.0,7.0,4.0]])

        newpts = np.zeros((1500,3))
        for i in range(1500):
            x = 5 * np.random.random() - 2.0
            y = 2 * np.random.random() + .25
            th = 0.0 #np.random.random()
            newpts[i] = [x, y, th]

        tg.new_graph()

        tg.draw_scatter(corners, 0,1,2,"s")
        tg.plot_points(newpts, 0,1,2, "s")
        tg.plot_points(g.pointArray, 0,1,2, "s")
        g.update(g.arrayToBelief(newpts))

        tg.new_graph()
        tg.draw_scatter(g.pointArray, 0,1,2, "s", recalc=False)

        hs = g.hist.plottable()
        tg.plot_points(hs, 0,1,2, "c")
        tg.plot_points(np.array([[means[0], means[1], -3.0]]), 0,1,2, "c")

        tg.mainloop()

