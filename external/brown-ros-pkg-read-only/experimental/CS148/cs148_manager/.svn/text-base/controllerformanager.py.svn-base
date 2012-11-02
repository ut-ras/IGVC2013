#!/usr/bin/env python
#import input
from manager import manager148

myMan = manager148()
curPID = 0
str = "/Applications/Chess.app/Contents/MacOS/Chess"
curPID = myMan.startProcess( str )
x = raw_input ('Type yes to kill')
if x == "yes":
	#myMan.printPID()
	myMan.killProcess( curPID )
else:
	print "Still running forever"