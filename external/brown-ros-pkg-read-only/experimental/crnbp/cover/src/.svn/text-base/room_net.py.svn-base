#!/usr/bin/env python
import roslib ; roslib.load_manifest('cover')
from cover.msg import Room2D, Room
from perfesser.msg import Pt
import numpy as np
import random
import string
import tgraph

class Node(object):
    """
    Lightweight class for extracting a network from a collection of robots.
    """
    def __init__(self, name, loc, rank=1000):
        self.name = name
        self.location = loc
        self.rank = rank
        self.root = False

        self.friends = []

        # This is used to test this node's position from some point.
        self.dist = 0
        self.distLoc = []

    def add(self, node):
        if node.name not in [ f.name for f in self.friends ] and \
                node.name != self.name:
            self.friends.append(node)

    def str(self):
        out = "%s " % (self.name,)
        out += "(%d) " % self.rank
        out += "(%.3f, %.3f) [" % tuple(self.location)
        for friend in self.friends:
            out += friend.name + ", "
        out += "]"
            
        return out

class RoomNet(object):
    def __init__(self, distance, sourceLocs):
        self.distance = distance

        # Random access for the Node objects we'll use.
        self.maps = {}

        # Random access for the source locations we're analyzing.
        self.locs = {}
        for l in sourceLocs:
            self.locs[l[0]] = l[1]


    def connect(self):
        """
        Find the connections possible for the input locations and the
        distance limit.
        """
        self.guess()
        # Randomly choose a "root"
        root = self.maps[sorted(self.maps.keys())[0]]

        looplimit = 0
        while not self.testConnected():
            self.clearRank()
            self.assignRank(root, 0)
            self.bestConnection()
            looplimit += 1
            if looplimit > 25:
                break
    

    def dist(self, pta, ptb):
        return sum([ (a - b)**2 for a,b in zip(pta,ptb) ])**0.5

    def clearRank(self):
        """
        Clears the rank of all the nodes.
        """
        for node in self.maps.values():
            node.rank = 1000

    def assignRank(self, node, rank):
        """
        Marches through a connected tree and assigns a rank to everyone.
        You'll know the graph is not connected if some nodes have no rank
        after this has been run on some node.

        This should only be run after clearRank() to avoid loops.
        """
        if node.rank >= 1000:
            node.rank = rank

            for friend in node.friends:
                self.assignRank(friend, rank + 1)

    def testConnected(self):
        """
        True if all the nodes have non-null (<1000) rank.
        """
        for node in self.maps.values():
            if node.rank >= 1000:
                return False
        return True

    def nearest(self, targetLoc):
        """
        Finds the point on the set of all unconnected nodes closest to the
        input location.
        """
        dlist = []
        for node in self.maps.values():
            if node.rank >= 1000:
                node.dist = self.dist(node.location, targetLoc)
                dlist.append(node)

        if not dlist:
            return False
        else:
            dlist.sort(key = lambda x: x.dist)

        return dlist[0]

    def bestConnection(self):
        """
        Finds the best unconnected connection possible.
        """
        if self.testConnected():
            return

        dlist = []
        for node in self.maps.values():
            if node.rank < 1000:
                nst = self.nearest(node.location)
                node.dist = self.dist(node.location, nst.location)
                dlist.append((node.dist, node, nst))
                
        if not dlist:
            return
        else:
            dlist.sort()

        if dlist[0][0] > self.distance:
            return
        else:
            dlist[0][1].add(dlist[0][2])


    def guess(self):
        # For each node we've got.
        for name in self.locs.keys():
            # Make a list of the nodes nearest to it.
            dlist = []
            for tname in self.locs.keys():
                dlist.append((self.dist(self.locs[name], self.locs[tname]), 
                              tname))
            # Sort the list.
            dlist.sort()

            # Add this node to the maps, if it's not already there.
            if name not in self.maps.keys():
                self.maps[name] = Node(name, self.locs[name])
            # If there are some neighbors, attach them.
            if len(dlist) > 1 and dlist[1][0] < self.distance:
                if dlist[1][1] not in self.maps.keys():
                    self.maps[dlist[1][1]] = \
                        Node(dlist[1][1], self.locs[dlist[1][1]])
                self.maps[name].add(self.maps[dlist[1][1]])
                self.maps[dlist[1][1]].add(self.maps[name])

            if len(dlist) > 2 and dlist[2][0] < self.distance and dlist[2][0] < dlist[1][0] * 2:
                if dlist[2][1] not in self.maps.keys():
                    self.maps[dlist[2][1]] = \
                        Node(dlist[2][1], self.locs[dlist[2][1]])
                self.maps[name].add(self.maps[dlist[2][1]])
                self.maps[dlist[2][1]].add(self.maps[name])

            # if len(dlist) > 3 and dlist[3][0] < self.distance and dlist[3][0] < dlist[1][0] * 2:
            #     if dlist[3][1] not in self.maps.keys():
            #         self.maps[dlist[3][1]] = \
            #             Node(dlist[3][1], self.locs[dlist[3][1]])
            #     self.maps[name].add(self.maps[dlist[3][1]])
            #     self.maps[dlist[3][1]].add(self.maps[name])

    def reciprocate(self):
        for node in self.maps.values():
            for friend in node.friends:
                if node.name not in [ f.name for f in friend.friends ]:
                    print "found one:", node.name, friend.name
                    friend.add(node)
    def unwind(self):
        """Returns a Room2D object containing all the connections."""
        out = Room2D()
        for node in self.maps.values():
            rm = Room(sourceName = node.name)
            for friend in node.friends:
                rm.names.append(friend.name)
                rm.locations.append(Pt(point=friend.location))
            out.robots.append(rm)

        return out

    def drawNeighbors(self):
        for name in self.maps.keys():
            node = self.maps[name]
            for friend in node.friends:
                tg.plot_lines(np.array([[node.location[0],node.location[1], 1.0],
                                        [friend.location[0], friend.location[1], 1.0]]),
                              0,1,2,"red")

if __name__ == '__main__':


    sourceLocs = [["a", [5.0, 5.0]]]
    for i in range(30):
        sourceLocs.append([ random.choice(string.letters[0:26]) + \
                                random.choice(string.letters[0:26]),
                            [10.0 * np.random.random(), 10.0 * np.random.random()]])

    tg = tgraph.Tgraph(500,500)
    tg.new_graph()


    pts = np.array( [ x[1] for x in sourceLocs ] )
    for x in sourceLocs:
        print x

    tg.draw_scatter(pts, 0, 1, 1, "s")

    roomnet = RoomNet(2.5, sourceLocs)
    roomnet.connect()

    roomnet.drawNeighbors()


    for node in roomnet.maps.values():
        if node.rank >= 1000:
            print node.name, node.rank, node.location
            tg.plot_lines(np.array([[node.location[0],node.location[1], 1.0],
                                    [0.0, 5.0, 1.0]]),
                          0,1,2,"green")

    roomnet.reciprocate()
    rm = roomnet.unwind()

    for r in rm.robots:
        print r.sourceName, r.names

    tg.mainloop()
