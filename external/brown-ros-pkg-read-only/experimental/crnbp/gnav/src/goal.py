#!/usr/bin/env python

class Goal(object):
    """
    Use this class to encapsulate a goal.  A goal is a target location,
    an optional type and an optional pointer to indicate some data
    associated with that goal.
    """
    def __init__(self, coords, gtype=""):
        self.coords = tuple(coords)
        # This is a pointer to some object associated with this goal.
        self.goalPtr = ""
        self.gtype = gtype

    def setCoords(self, coords):
        """
        Use this as a tidy way to change a goal's coordinates.
        """

        self.coords = coords

    def setPtr(self, newPtr):
        """
        Attaches some data to the goal.  For example, a goal might
        have a force associated with it, and you can use this pointer
        to keep track of that force.
        """
        self.goalPtr = newPtr

    def str(self):
        """
        A pretty-printer for a goal.
        """
        out = "{0}:".format(self.gtype) if self.gtype else ""
        out += "{0}".format(repr(self.coords))
        out += "[{0}]".format(str(self.goalPtr)) if self.goalPtr else ""
        return out

class GoalStack(object):
    """
    This class keeps track of a stack of goals.  If a goal has to be
    pre-empted, say because of an obstacle, another goal may be pushed
    onto the stack above it and the robot can operate on that one
    instead.  This stack also supports modifying goals that are
    already on the stack.  You can modify a goal's coordinates or the
    associated object, but the type is considered immutable.  If that
    needs to change, just delete it and create another Goal() object.

    The goal objects themselves are kept in a dict object called
    goals, and their order is kept in an array of integers called
    stack.  The keys to the goals dict are integers indicating the
    order in which they were added.
    """
    def __init__(self):
        self.goals = {}
        self.stack = []
        self.index = 0

    def push(self, g):
        """
        Add a Goal object to the stack.  Returns the index of the new
        object.  These are awarded in order, so it indicates how many
        goals have been added so far, but it does not tell you how
        many are still on the stack or what position this goal is.
        But save the index because it can be used to update the Goal
        object if something needs to change.
        """
        assert isinstance(g, Goal), "g is not a Goal: %s" % `g`

        self.index += 1
        self.stack.append(self.index)
        self.goals[self.index] = g
        return self.index

    def pop(self, index = 0):
        """
        Removes a goal from the stack.  With no argument, removes the
        top().
        """
        if self.goals:
            if (index == 0):
                return self.goals.pop(self.stack.pop())
            else:
                self.stack.remove(index)
                return self.goals.pop(index)
        else:
            return False

    def top(self):
        """
        The top() goal is the one the robot should be working
        towards.  This method finds that goal, but doesn't change it
        or move it.
        """
        if self.goals:
            return self.goals[self.stack[-1]]
        else:
            return False

    def update(self, index, newg):
        """
        Changes the coordinates in a Goal object.  The input is a Goal
        object and the coordinates from this input object are
        transferred to the Goal object already on the stack.
        """
        if index in self.stack:
            self.goals[index].setCoords(newg.coords)
            return self.goals[index]
        else:
            return False

    def updatePtr(self, index, newPtr):
        """
        Changes the data pointer for a particular Goal object already
        on the goal stack.
        """
        if index in self.stack:
            self.goals[index].setPtr(newPtr)
            return self.goals[index]
        else:
            return False

    def str(self):
        """
        Formatter for printing.
        """
        out = ""
        for i in reversed(self.stack):
            out += "{0}:{1}\n".format(i, self.goals[i].str())
        return out[0:-1]


# Testing suite
if __name__ == '__main__':

    import unittest
    class TestGoals(unittest.TestCase):
        def setUp(self):
            self.gs = GoalStack()
            self.one = self.gs.push(Goal((1,2,1)))
            self.two = self.gs.push(Goal((1,2,2)))
            self.three = self.gs.push(Goal((1,2,3)))
            self.four = self.gs.push(Goal((1,2,4)))

        def testaStack(self):
            self.assertEqual(self.four, 4)

            testString = "4:(1, 2, 4)\n3:(1, 2, 3)\n2:(1, 2, 2)\n1:(1, 2, 1)"
            self.assertEqual(self.gs.str(), testString)

            testString = "4:(1, 2, 4)\n3:(3, 3, 3)\n2:(1, 2, 2)\n1:(1, 2, 1)"
            self.gs.update(self.three, Goal((3,3,3)))
            self.assertEqual(self.gs.str(), testString)

            testString = "4:(1, 2, 4)\n3:(3, 3, 3)\n1:(1, 2, 1)"
            two = self.gs.pop(self.two)
            self.assertEqual(two.str(), "(1, 2, 2)")
            self.assertEqual(self.gs.str(), testString)

            testString = "(1, 2, 4)"
            self.assertEqual(self.gs.top().str(), testString)

            testString = "1:(1, 2, 1)"
            self.gs.pop()
            self.gs.pop()
            self.assertEqual(self.gs.str(), testString)

            testString = "5:b:(2.2999999999999998, 3.1000000000000001, 3.1428571428571428)\n1:(1, 2, 1)"
            self.five = self.gs.push(Goal((2.3, 3.1, 22.0/7.0), gtype="b"))
            self.assertEqual(self.gs.str(), testString)

            testString="5:b:(2.2999999999999998, 3.1000000000000001, 3.1428571428571428)[hello]\n1:(1, 2, 1)"
            self.gs.updatePtr(5,"hello")
            self.assertEqual(self.gs.str(), testString)

            self.gs.pop()
            self.gs.pop()
            self.assertFalse(self.gs.pop())
            self.assertFalse(self.gs.updatePtr(5,"good-bye"))

    suite = unittest.TestLoader().loadTestsFromTestCase(TestGoals)
    unittest.TextTestRunner(verbosity=3).run(suite)

#    unittest.main()

