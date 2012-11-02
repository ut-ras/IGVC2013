#!/usr/bin/env python
import roslib; roslib.load_manifest('block_build_manager')
import rospy
import actionlib

from block_build_msgs.msg import *
from object_manager.msg import *
from object_manager.srv import *
from geometry_msgs.msg import *
from std_msgs.msg import *

from std_srvs import *

""" 
  Author : Jihoon Lee
  Date   : Feb. 2012

  BuildManager.py
"""
COMMAND_NAME = "/blocknlp/command"
PICK_COMMAND_NAME = "/blocknlp/pick"
PLACE_COMMAND_NAME = "/blocknlp/place"
OBJECT_LIST_TOPIC = "/blocknlp/object_list"
OBJECT_UPDATE_TOPIC_NAME = "/blocknlp/update_object"
UPDATE_TABLE_TOPIC_NAME = "/blocknlp/update_table"
SLU_TOPIC_NAME = "/blocknlp/nl_output"

class BuildManager:

  command_server = None
  pick_client = None
  place_client = None
  start_srv_server = None
  slu_command_listner = None

  object_update_pub = None
  table_update_pub = None

  inProcessCommand = False

  object_inhand = None

  
  def __init__(self):
    self.command_server = actionlib.SimpleActionServer(COMMAND_NAME,block_build_msgs.msg.CommandAction,self.commandCallback,False)
    self.pick_client = actionlib.SimpleActionClient(PICK_COMMAND_NAME,block_build_msgs.msg.PickCommandAction)
    self.place_client = actionlib.SimpleActionClient(PLACE_COMMAND_NAME,block_build_msgs.msg.PlaceCommandAction)

    self.slu_command_listner = rospy.Subscriber(SLU_TOPIC_NAME,SLUCommand,self.sluCommandListener)

    self.object_update_pub = rospy.Publisher(OBJECT_UPDATE_TOPIC_NAME,Object)
    self.table_update_pub = rospy.Publisher(UPDATE_TABLE_TOPIC_NAME,Empty)


  def processCommand(self,req):
    if req.command == 0:
      self.pickup_by_id(req.obj_num)
    elif req.command == 1:
      self.putdown(0,[req.x,req.y])
    elif req.command == 2:
      self.initializeRobot()
    return True

  def initializeRobot(self):
    g = PickCommandGoal()
    g.number = -3

    rospy.loginfo("Send initialize robot message")
    self.pick_client.send_goal(g)
    self.pick_client.wait_for_result()

    if self.pick_client.get_result().result:
      rospy.loginfo("Complete")
    else:
      rospy.loginfo("Failed")

  def slu_command_parser(self,msg):
    putdown = self.putdown
    pickup = self.pickup_by_id

    eval(msg.command)

    return msg

  def pickup_by_id(self, id):
    # Create the goal object
    g = PickCommandGoal()
    g.number = -2

    obj_list = rospy.wait_for_message(OBJECT_LIST_TOPIC,ObjectList,10.0)

    self.inProcessCommand = True
   
    foundObject = False
    for obj in obj_list.object_list:
      if id == obj.id:
        g.object = obj
        foundObject = True
        break
   
    if not foundObject:
        return False

    rospy.loginfo("Send pick goal")
    self.pick_client.send_goal(g)
    self.pick_client.wait_for_result()
    if self.pick_client.get_result().result:
      rospy.loginfo("Complete")
      new_obj = Object()
      new_object = g.object
      new_object.header.frame_id = "/r_gripper_tool_frame"
      new_object.inhand = True
      new_object.pose = Pose()
      self.object_inhand = new_object
      self.object_update_pub.publish(new_object)
    else:
      rospy.loginfo("Fail")
    self.inProcessCommand = False

  def pickup(self,num):
    g = PickCommandGoal()
    g.number = -2

    obj_list = rospy.wait_for_message(OBJECT_LIST_TOPIC,ObjectList,10.0)

    self.inProcessCommand = True

    if num > len(obj_list.object_list):
      return False
    else:
      g.object = obj_list.object_list[num]

    rospy.loginfo("Send pick goal")
    self.pick_client.send_goal(g)
    self.pick_client.wait_for_result()
    if self.pick_client.get_result().result:
      rospy.loginfo("Complete")
      new_obj = Object()
      new_object = g.object
      new_object.header.frame_id = "/r_gripper_tool_frame"
      new_object.inhand = True
      new_object.pose = Pose()
      self.object_inhand = new_object
      self.object_update_pub.publish(new_object)
    else:
      rospy.loginfo("Fail")
#    self.inProcessCommand = False

  def putdown(self,id,location):
    #if self.object_inhand == None:
    #  rospy.loginfo("No object in hand")
    #  return True

    self.inProcessCommand = True
    placeGoal = PlaceCommandGoal()
    placeGoal.x = location[0]
    placeGoal.y = location[1]
    placeGoal.z = 0
    placeGoal.orientation = Quaternion(0,0.707,0,0.707)
    placeGoal.precise_place = True            
    print(str(placeGoal))
    
    rospy.loginfo("Send place goal")
    self.place_client.send_goal(placeGoal)
    self.place_client.wait_for_result()
    if self.place_client.get_result().result:
      rospy.loginfo("Complete")
    else:
      rospy.loginfo("Fail")

    self.object_inhand.inhand = False
    self.object_update_pub.publish(self.object_inhand)
    self.object_inhand = None

    self.inProcessCommand = False

  def sluCommandListener(self,msg):
    rospy.loginfo("in SLU command listener msg :")
    rospy.loginfo(msg.command)

    # parse Message from SLU
    self.slu_command_parser(msg)
    
  def commandCallback(self,goal):
    if goal.command == 99:
      rospy.loginfo("It works!")
    else:
      self.processCommand(goal)

    r = CommandResult()
    r.result = True
    self.command_server.set_succeeded(r)


  def spin(self):
    self.command_server.start()
    rospy.loginfo('Wait for pick client')
#    self.pick_client.wait_for_server()
#    rospy.loginfo('Wait for place client')
#    self.place_client.wait_for_server()

    rospy.loginfo('initialized')
    while not rospy.is_shutdown():
      if not self.inProcessCommand:
        t = Empty()
        self.table_update_pub.publish()
      rospy.sleep(2)


if __name__ == '__main__':
  rospy.init_node('block_build_manager')

  bm = BuildManager()
  bm.spin()

