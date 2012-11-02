#!/usr/bin/env python
from model_create import *
import time
import roslib
import rospy
import force

rospy.init_node("model_create")


room = RoomModel(0.1, 500,500, 3.0, 3.0, debug=True)

rob = CreateModel(-2.0,-1.0,0.0,color=2,name="rob", debug=True)
rob.forceList.addForce(force.ForceVector((2.0,0.75,-1.8)))

rob.forceList.addForce(force.ForceField((0.5,0.5)))
rob.forceList.addForce(force.ForceField((-0.5,0.5)))
rob.forceList.addForce(force.ForceField((0.5,-0.5)))
rob.forceList.addForce(force.ForceField((-0.5,-0.5)))
rob.forceList.addForce(force.ForceField((1.5,1.5)))
rob.forceList.addForce(force.ForceField((-1.5,1.5)))
rob.forceList.addForce(force.ForceField((1.5,-1.5)))
rob.forceList.addForce(force.ForceField((-1.5,-1.5)))
rob.forceList.addForce(force.ForceField((1.5,0.5)))
rob.forceList.addForce(force.ForceField((-1.5,0.5)))
rob.forceList.addForce(force.ForceField((1.5,-0.5)))
rob.forceList.addForce(force.ForceField((-1.5,-0.5)))
rob.forceList.addForce(force.ForceField((0.5,1.5)))
rob.forceList.addForce(force.ForceField((-0.5,1.5)))
rob.forceList.addForce(force.ForceField((0.5,-1.5)))
rob.forceList.addForce(force.ForceField((-0.5,-1.5)))


room.addRobot(rob)

pete = CreateModel(-1.0,-1.0,0.0,color=90, name="pete", debug=True)
pete.forceList.addForce(force.ForceVector((2.0,1.25,-2.5)))
pete.forceList.addForce(force.ForceField((0.25,0.15)))
pete.forceList.addForce(force.ForceField((0.5,0.5)))
pete.forceList.addForce(force.ForceField((-0.5,0.5)))
pete.forceList.addForce(force.ForceField((0.5,-0.5)))
pete.forceList.addForce(force.ForceField((-0.5,-0.5)))
pete.forceList.addForce(force.ForceField((1.5,1.5)))
pete.forceList.addForce(force.ForceField((-1.5,1.5)))
pete.forceList.addForce(force.ForceField((1.5,-1.5)))
pete.forceList.addForce(force.ForceField((-1.5,-1.5)))
pete.forceList.addForce(force.ForceField((1.5,0.5)))
pete.forceList.addForce(force.ForceField((-1.5,0.5)))
pete.forceList.addForce(force.ForceField((1.5,-0.5)))
pete.forceList.addForce(force.ForceField((-1.5,-0.5)))
pete.forceList.addForce(force.ForceField((0.5,1.5)))
pete.forceList.addForce(force.ForceField((-0.5,1.5)))
pete.forceList.addForce(force.ForceField((0.5,-1.5)))
pete.forceList.addForce(force.ForceField((-0.5,-1.5)))
room.addRobot(pete)




n = 0
while (not raw_input("another point? (ret = yes)")):
    n += 1

    for robot in room.robots:
            robot.curTwist = \
                robot.forceList.sumForces((robot.x, robot.y, robot.th),
                                          robot.curTwist)
            robot.twist(robot.curTwist)
        
    room.updateRoom()


