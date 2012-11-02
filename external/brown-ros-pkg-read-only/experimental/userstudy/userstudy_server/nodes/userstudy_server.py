#!/usr/bin/env python
import roslib; roslib.load_manifest('userstudy_server')
import rospy

import sys
from os import system

"""
  Author : Jihoon Lee
  Date   : Jul. 2012
"""

from userstudy_msgs.msg import *

class UserStudyServer:
  
  WEB_ACTION_NAME = "/userstudy/web"
  CLIENT_TOPIC_NAME = "/userstudy/client"

  web_listener    = None
  client_listener = None

  WAIT              = 0
  START             = 0
  BACKEND_READY     = 1
  EXPERIMENT_START  = 2
  EXPERIMENT_END    = 3
  
  def __init__(self):
    self.server = actionlib.SimpleActionServer(WEB_ACTION_NAME,WebAction,self.webListener,False)
    self.client_listener = rospy.Subscriber(TOPIC_NAME,Status,self.clientListener)
    self.status = WAIT 

  def webListener(self,goal):
    
    if self.status != WAIT:
      self.sendError()
      r = WebResult()
      r.result = False
      self.server.set_succeeded(r)
      return
    else:
      self.status = START

    # launch the back-end interface
    self.launch(goal.filename)

    # wait for interface
    self.waitForInterface()

    # notify web that back-end is ready.
    self.sendFeedback('ready')

    # wait for ready 
    self.waitForReady()

    # wait for end 
    result = self.waitForEnd()

    self.server.set_succeeded(r)
    self.status = WAIT
    
  def clientListener(self,msg):
    rospy.loginfo('Client')
    if msg.status == 'ready':
      self.status = BACKEND_READY
    elif msg.status == 'feedback':
      self.processFeedback()
    elif msg.status == 'end':
      self.status = EXPERIMENT_END

  def launch(filename):
    try:
      file = open(filename,'r')
      rospy.loginfo("launching %s" % (filename,)
      system('roslaunch %s &' % (filename, ))
    except:
      e = sys.exc_info()[1]
      print "Problem: %s" % (e, )

  def spin(self):
    rospy.spin()

if __name__== '__main__':
  rospy.init_node('userstudy_server')

  server = UserStudyServer()
  rospy.loginfo('Initialized')
  server.spin()
  rospy.loginfo('Bye Bye')

