#!/usr/bin/env python
#
# A collection of functions for expressing where a robot and its pals
# should go.
import numpy as np
import math
import copy


class OGrid(object):
    """
    A spatially-related grid.
    """
    def __init__(self, nx, ny, ox, oy, rx, ry):
        """
        nx, ny = number of points
        ox, oy = location of origin
        rx, ry = resolution in x and y directions
        """
        self.nx = nx
        self.ny = ny
        self.ox = ox
        self.oy = oy
        self.rx = rx
        self.ry = ry

        self.oGrid = np.zeros((self.nx, self.ny))
        self.gradGrid = [ np.zeros((self.nx, self.ny)),
                          np.zeros((self.nx, self.ny)) ]
        self.gridChanged = True

    def add(self, pos):
        try:
            self.oGrid[ int((pos[0] - self.ox)/self.rx), 
                        int((pos[1] - self.oy)/self.ry) ] += 1.0
            self.gridChanged = True
        except:
            pass

    def get(self, pos):
        try:
            return self.oGrid[ int((pos[0] - self.ox)/self.rx), 
                               int((pos[1] - self.oy)/self.ry) ] 
        except:
            return float('NaN')

    def gradient(self, pos):
        """
        Returns a vector pointing at the steepest uphill direction."
        """
        ix = int((pos[0] - self.ox)/self.rx)
        iy = int((pos[1] - self.oy)/self.ry)

        # A point out of bounds could be a measurement error, or it could
        # be new territory.  For now, we'll assume the former.
        ix = max(0, min(self.nx - 1, ix))
        iy = max(0, min(self.ny - 1, iy))

        if self.gridChanged:
            self.gradGrid = np.gradient(self.oGrid)
            self.gridChanged = False

        out = [ self.gradGrid[0][ix,iy], self.gradGrid[1][ix,iy] ]
        sumsq = sum([ x**2 for x in out ])**0.5

        return tuple([ x/sumsq for x in out ])

class Phi(object):
    """
    A set of functions to use to establish goals for a set of robots.
    The motion of the robots is handled in an n-dimensional
    configuration space, where n is two times the number of robots.
    """
    def __init__(self, name, aw, sp, av, se, maxDist, resolution = 0.1):
        """
        Initialize the function with the four coefficients for the
        four components of action: away (robots move away from each
        other), space (robots move toward better spacing), avoid
        (robots avoid known obstacles), and seek (robots move toward
        an area of interest).
        """
        self.name = name
        self.aw = aw
        self.sp = sp
        self.av = av
        self.se = se
        self.maxDist = maxDist
        self.targetDist = self.maxDist / 2.0

        # A list of obstacles already encountered.  Tracking obstacles
        # is important not only for avoiding them in the future, but
        # also for the seek behavior, which has to take them into
        # account when deciding what territory is open because it
        # hasn't been explored and what territory is open because it's
        # protected by obstacles.
        self.obstacles = []

        self.resolution = resolution

        # Any obstacle closer than the square root of this number to
        # another obstacle is deemed to be 'close', and is not added
        # to the obstacle list.
        self.nearDistance = (self.resolution)**2

        # An occupancy grid.  Use this to keep track of places we've
        # been and to inform us about where we ought to recommend
        # other robots go.
        self.ndim = int(1.0 / self.resolution)

        self.oGrid = OGrid(self.ndim, self.ndim, 0.0, 0.0, 
                           self.resolution, self.resolution)

    def updateOccupancy(self, pos):
        """
        Adds to an entry in the occupancy grid.
        """
        self.oGrid.add(pos)


    def occupancyDirection(self, pos):
        """
        Given a location, returns a unit vector in a direction worth
        going if you want to visit unvisited territory.
        """
        return self.oGrid.gradient(pos)


    def near(self, ppt, qpt):
        if self.nearDistance <  sum([ (p - q)**2 for p,q in zip(ppt, qpt) ]):
            return False
        else:
            return True

    def addObstacle(self, pos):
        """
        This is called when a bump is detected.  Checks to see whether
        we already know about this obstacle, and if not, we add it to
        the list of obstacles we know about.
        """
        if self.obstacles:
            alreadyFound = False
            for obs in self.obstacles:
                if self.near(pos[0:2], obs[0:2]):
                    alreadyFound = True
                    break
            if not alreadyFound:
                #print self.name, "found another one!", pos
                self.obstacles.append(pos)
        else:
            self.obstacles = [ pos ]

    def away(self, pos):
        """
        Find the 'away' vector in configuration space by making each
        robot a repeller.  
        """
        if len(pos) < 3:
            return [ 0.0 ] * len(pos)

        if len(pos) < 7:
            exp = 5
        else:
            exp = 2

        try:
            plist =  [ (pos[i], pos[i+1]) for i in range(0, len(pos), 2) ]
        except:
            print self.name, pos

        out = []
        for p in plist:
            fxsum = 0.0
            fysum = 0.0
            for q in plist:
                fx = q[0] - p[0]
                fy = q[1] - p[1]
                d = ((p[0] - q[0])**2 + (p[1] - q[1])**2)**0.5
                
                if d > 0:
                    if d < 0.5 * self.targetDist:
                        fm = -1.0/d**2
                    elif d < 1.5 * self.targetDist:
                        fm = 0.03 * (d - self.targetDist)
                    else:
                        fm = (7.0*(d - self.targetDist))**exp

                    fxsum += fm * fx/d
                    fysum += fm * fy/d

            out.append(fxsum)
            out.append(fysum)

        return out

        # sumsq = sum([ x**2 for x in out ])**0.5

        # return [ x/sumsq for x in out ]

    def dist(self, pos): 
        """
        Input is a vector of xs and ys in real space.  Output is a
        vector of distances.  The minimum distance between two points
        in the input vector is out[-1] and the max is out[0].
        """
        if len(pos) < 3:
            return [0.0]

        out = []
        for i in range(0,len(pos),2):
            for j in range(i + 2,len(pos),2):
                out.append(((pos[i] - pos[j])**2 + \
                                (pos[i+1] - pos[j+1])**2)**0.5)
        out.sort()
        out.reverse()
        return out       

    def cross(self, vecs):
        """
        Compute the n-dimensional cross product for a list of n-1
        n-dimensional vectors using the determinant method:
        
        a11     a12     a13     ... a1n
        a21     a22     a23     ... a2n
        ...
        a(n-1)1 a(n-1)2 a(n-1)3 ... a(n-1)n

        an1 = -det(square matrix created by excluding first column)
        an2 = det(square matrix created by excluding second column)
        an3 = -det(square matrix created by excluding third column)
        an4 = det(square matrix created by excluding fourth column)
        ...
        """
        ndim = len(vecs) + 1
        assert len(vecs[0]) == ndim

        arr = np.array(vecs)

        out = []
        for i in range(ndim):
            dims = range(ndim)
            dims.remove(i)
            val = np.linalg.det(np.take(arr, dims, 1))
            val = -val if (i % 2) == 1 else val
            out.append(val)

        return out  

    def space(self, cpoint, away):
        """
        Given cpoint, a point in configuration space made up of n 2-d
        points, and 'away', the efficient vector to move each point
        away from each other, calculate a move for the first two
        coordinates that is orthogonal to the away vector.  The hope
        is that this is a direction that doesn't move the robots away
        from each other but towards a more uniform spacing.

        We do this by calculating the cross product of 'away' with
        unit vectors in the direction of all the other coordinates.
        """
        sumsq = sum([ y**2 for y in away ])**0.5
        if sumsq == 0:
            vecs = away
        else:
            vecs = [ x/sumsq for x in away ]

        ndim = len(cpoint)
        if ndim < 3:
            return np.array([ 0.0 ] * ndim)

        for i in range(2,ndim):
            row = np.array([ 0.0 ] * ndim)
            row[i] = 1.0
            vecs = np.vstack((vecs, row))

        tentative = self.cross(vecs)

        minDist = lambda(x): np.var(self.dist(x))
        origMin = minDist(cpoint)

        # For convenience, we want the scale of the space vector to be 
        # comparable to the scale of the away vector.

        if minDist([ y + g for y,g in zip(cpoint,tentative)]) < origMin:
            out = [ -y * sumsq for y in tentative ]
        else:
            out = [ y * sumsq for y in tentative ]

        # 3-d cross product
        return tuple(out)

    def avoid(self, cpoint):

#        if self.obstacles:
#            print self.name, "sees obstacles:", self.obstacles
        out =  []

        for i in range(0, len(cpoint), 2):
            fxsum = 0.0
            fysum = 0.0
            for obs in self.obstacles:
                fx = obs[0] - cpoint[i]
                fy = obs[1] - cpoint[i + 1]
                d = (fx**2 + fy**2)**0.5
                
                if 0 < d: # < self.maxDist/10.0 :
                    fm = - 1.0/d**3

                    fxsum += fm * fx/d
                    fysum += fm * fy/d

            out.append(fxsum)
            out.append(fysum)

        return out

        # # Normalize, then return
        # sumsq = sum([ x**2 for x in out ])**0.5

        # if sumsq > 0.0:
        #     return [ x/sumsq for x in out ]
        # else:
        #     return out

    def seek(self, cpoint):

        # There are some points of interest.  Make them into
        # attractors for robots who aren't past them.
        attractors = []
        out = []

        for i in range(0, len(cpoint), 2):
            fxsum = 0.0
            fysum = 0.0

            if cpoint[i] > 8.4 and 5.0 < cpoint[i+1] < 7.6:
                attractors.append((8.5, 3.0))
                attractors.append((8.5, 8.0))
            elif cpoint[i] > 8.4 and cpoint[i+1] > 4.1:
                attractors.append((8.5, 3.5))
            elif cpoint[i] > 8.2 and 7.5 < cpoint[i+1] < 7.8:
                attractors.append((8.9, 9.0))

            if cpoint[i] > 4.9 and 3.5 < cpoint[i+1] < 4.2:
                attractors.append((4.5, 4.0))

            if cpoint[i] < 4.6 and cpoint[i+1] < 4.5:
                attractors.append((3.0, 7.0))

            for attractor in attractors:
                fx = attractor[0] - cpoint[i]
                fy = attractor[1] - cpoint[i + 1]
                d = (fx**2 + fy**2)**0.5
                
                if d > 0.0:
                    fm =  1.0 + 0.5*math.e**(-d)
                    fxsum += fm * fx/d
                    fysum += fm * fy/d

            out.append(fxsum)
            out.append(fysum)

        return( out )

 # The it's not the location that is important, but the path to the
 # location.  Useful empty space will be empty cells bordering
 # occupied cells, where there aren't previously recorded bumps.  But
 # the route between here and there must be through previously
 # occupied cells.  So fan out from the occupied cell and look for the
 # first unoccupied cell, then discard all the paths that have bumps
 # in the way.


        # out = []
        # for i in range(0, len(cpoint), 2):
        #     d = self.occupancyDirection((cpoint[i], cpoint[i+1]))
        #     out.append(d[0])
        #     out.append(d[1])

        # # Normalize, then return
        # sumsq = sum([ x**2 for x in out ])**0.5

        # if sumsq > 0.0:
        #     return [ x/sumsq for x in out ]
        # else:
        #     return out


    def phi(self, cpoint):
        """
        Using the input cpoint of points in configuration space,
        formulate a suggestion about where we would like to move and
        where we think the others in this cpoint should move, too.  The
        first column (or columns) is assumed to correspond to ourself,
        the bookkeeping for the other columns is left to the caller.
        """
        ## Find the four primary 'directions' in configuration space.
        away = self.away(cpoint)
        space = self.space(cpoint, away)
        avoid = self.avoid(cpoint)
        seek = self.seek(cpoint)

        ## Find distances between all the points.
        dists = self.dist(cpoint)

        # Add them.
        return [self.aw * w + self.sp * x + self.av * y + self.se * z \
                    for w,x,y,z in zip(away, space, avoid, seek) ]

