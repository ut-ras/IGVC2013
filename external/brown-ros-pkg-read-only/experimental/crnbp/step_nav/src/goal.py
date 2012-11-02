#!/usr/bin/env python
# Use this class to encapsulate a goal.  A goal is a target location,
# an optional type and an optional pointer to indicate some data
# associated with that goal.
class Goal(object):
    def __init__(self, coords, gtype=""):
        self.coords = tuple(coords)
        # This is a pointer to some object associated with this goal.
        self.goalPtr = ""
        self.gtype = gtype
        self.type = type

    # Use this as a tidy way to change a goal's coordinates.
    def setCoords(self, coords):
        self.coords = coords

    # Attaches a 
    def setPtr(self, newPtr):
        self.goalPtr = newPtr

    def str(self):
        out = "{0}:".format(self.gtype) if self.gtype else ""
        out += "{0}".format(repr(self.coords))
        out += "[{0}]".format(str(self.goalPtr)) if self.goalPtr else ""
        return out

# This class keeps track of a stack of goals.  If a goal has to be
# pre-empted, say because of an obstacle, another goal is pushed onto
# the stack above it and we operate on that one instead.  This stack
# also supports modifying goals that are already on the stack.  You
# can modify a goal's coordinates or the associated object, but the
# type is considered immutable.  If that needs to change, just delete
# it and create another Goal() object.

# This class keeps track of a stack of goals.  The goal objects
# themselves are kept in a dict object called goals, and their order
# is keps in an array of integers called stack.  The keys to the goals
# dict are integers indicating the order in which they were added.

class GoalStack(object):
    def __init__(self):
        self.goals = {}
        self.stack = []
        self.index = 0

    def push(self, g):
        self.index += 1
        self.stack.append(self.index)
        self.goals[self.index] = g
        return self.index

    # pop and remove can be combined if we equip pop() with an
    # optional argument
    def pop(self, index = 0):
        if self.goals:
            if (index == 0):
                return self.goals.pop(self.stack.pop())
            else:
                self.stack.remove(index)
                return self.goals.pop(index)
        else:
            return False

    # We operate on the top goal while it is still on the stack.  That
    # way, if another goal is pushed on (asynchronously), the overall
    # operation remains the same.
    def top(self):
        if self.goals:
            return self.goals[self.stack[-1]]
        else:
            return False

    def update(self, index, newg):
        if index in self.stack:
            self.goals[index].setCoords(newg.coords)
            return self.goals[index]
        else:
            return False

    def updatePtr(self, index, newPtr):
        if index in self.stack:
            self.goals[index].setPtr(newPtr)
            return self.goals[index]
        else:
            return False

    def str(self):
        out = ""
        for i in reversed(self.stack):
            out += "{0}:{1}\n".format(i, self.goals[i].str())
        return out[0:-1]

    def prin(self):
        print self.str();

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

