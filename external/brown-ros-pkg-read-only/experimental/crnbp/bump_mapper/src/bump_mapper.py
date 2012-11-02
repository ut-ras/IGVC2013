#!/usr/bin/env python
#
# Listens for a bump.  If it hears one, fires off an estimate of the
# obstacle position.
#
# As of 12/1/11, this has not been extensively tested.
import roslib ; roslib.load_manifest('bump_mapper')
import rospy
from irobot_create_2_1a.msg import SensorPacket
from geometry_msgs.msg import Twist
from perfesser_guesser import *
from perfesser.msg import Belief, Pt
import math

class BumpMapper(object):
    """
    Tracks position estimate beliefs, listens for bumps.  When it
    hears a bump, estimates the position of the obstacle that caused
    it and fires off a belief about it.
    """
    def __init__(self, name="bump_mapper", pub=False, radius=0.25):
        self.name = name
        self.bpub = pub
        self.robotRadius = radius

        self.waitTime = 0.0

        self.curLocation = Belief()
        self.curVelocity = Twist()
        self.velTimestamp = rospy.Time.now().to_sec()

        self.guesser = Guesser(name = self.name,
                               periods = (0.0, 0.0, 2*math.pi),
                               data_type = "location", num_bins = (1,1,1),
                               data_sub_type = "obstacle")

    def trackVelocity(self, req):
        """
        Records the command controlling the robot velocity.
        """
        self.curVelocity = req
        self.velTimestamp = rospy.Time.now().to_sec()

    def trackPosition(self, req):
        """
        Records the current robot position, or rather the current
        robot's belief about its position..  That's all.
        """
        assert isinstance(req, Belief)
        self.curLocation = req

    def checkBump(self, req):
        """
        Parses a SensorPacket for the bump and if it's there, fires
        off a message about it.  There are some provisions to make
        sure one bump doesn't trigger too large a number of responses.
        """
        if rospy.Time.now().to_sec() < self.waitTime:
            return

        if req.bumpLeft and req.bumpRight:
            direction = 0.0
        elif req.bumpLeft:
            direction = 1.0
        elif req.bumpRight:
            direction = -1.0
        else:
            return

        # Should consider using the Perfesser here for the means() and stds()
        bumpEst = Belief(source_stamp = self.curLocation.header.stamp,
                         sender = self.name,
                         source_data = self.curLocation.source_data,
                         data_type = self.guesser.data_type,
                         data_sub_type = self.guesser.data_sub_type,
                         pers = self.guesser.periods)
        bumpEst.points = []

        for p in self.curLocation.points:

            # Estimate the point on the robot's perimeter where the bump was
            # felt.
            psiCorr = p.point[2] + direction * 0.7
            # Calculate the opposite-facing direction.
            phiCorr = (psiCorr + math.pi) % (2 * math.pi)
            phiCorr -= (2 * math.pi) if psiCorr > math.pi else 0.0

            bumpEst.points.append( \
                Pt(point=(p.point[0] + self.robotRadius * math.cos(psiCorr),
                          p.point[1] + self.robotRadius * math.sin(psiCorr),
                          phiCorr)))

        self.guesser.newPoints(bumpEst.points)
        bumpEst.means = self.guesser.means()
        bumpEst.stds = self.guesser.stds()

        if self.bpub:
            bumpEst.header.stamp = rospy.Time.now()
            self.bpub.publish(bumpEst)

        self.waitTime = rospy.Time.now().to_sec() + 5.0
