#!/usr/bin/env python
#import roslib; roslib.load_manifest('cs148_manager')
#import rospy
import subprocess
import os, signal
import time

myPID = 2

class manager148():


	myPID = 0

	def startProcess(e, str ) :
    		p = subprocess.Popen(str)
    		myPID = p.pid
   	 	#time.sleep(2.0)
    		#os.kill(p.pid, signal.SIGKILL)
		return myPID

	def killProcess(e, curPID) :
    		time.sleep(2.0)
    		os.kill(curPID, signal.SIGKILL)

	def printPID(e) :
		print myPID


#if __name__ == '__main__':
 #   try:
        #import sys
    #if (len(sys.argv) > 1):
    #    startProj(sys.argv[1], sys.argv[2])

	

   #  except rospy.ROSInterruptException: pass