#!/usr/bin/python
from irobot import Create
import cwiid
from time import sleep
from models import *
from math import sqrt,fabs
from pickle import dump,load

def cos(model, sample):
	a = 0.0
	b = 0.0
	c = 0.0
	for i in xrange(0,30):
		for j in xrange(0,3):
			a = a + model[i][j]*sample[i][j]
			b = b + model[i][j]*model[i][j]
			c = c + sample[i][j]*sample[i][j]
	return 1.0 - (a/(sqrt(b)*sqrt(c)))
		

#create = Create("/dev/rfcomm0")
create = Create()

a = cwiid.Wiimote()
allLEDS = cwiid.LED1_ON ^ cwiid.LED2_ON ^ cwiid.LED3_ON ^ cwiid.LED4_ON
a.led = allLEDS

a.rpt_mode = cwiid.RPT_ACC ^ cwiid.RPT_BTN

cal = a.get_acc_cal(0)
zero = cal[0]
one = cal[1]

print zero,one

create.start()

def leftAction():
	create.turn(-50)
	return 'left'

def rightAction():
	create.turn(50)
	return 'right'

def forwardAction():
	create.tank(250,250)
	return 'forward'

def stopAction():
	create.brake()

counts = [0,0,0]
models = [left,forward,right]
mode = cwiid.BTN_A
train = False
save = ''
destroy = False

try:

	obs = []
	while(1):
		sleep(0.03)

		x = float(a.state['acc'][0] - zero[0]) / float(one[0]-zero[0])
		y = float(a.state['acc'][1] - zero[1]) / float(one[1]-zero[1])
		z = float(a.state['acc'][2] - zero[2]) / float(one[2]-zero[2])

		#0.037037037037 -0.964285714286 -0.037037037037
		if (fabs(x) < .1 and fabs(y + 1) < .1 and fabs(z) < .1):
			stopAction()

		buttons = a.state['buttons']

		if ((buttons & cwiid.BTN_A) 
                	or (buttons & cwiid.BTN_LEFT)  
			or (buttons & cwiid.BTN_UP) 
			or (buttons & cwiid.BTN_RIGHT)):
				obs.append([x,y,z])

		if (buttons & cwiid.BTN_A):
			mode = cwiid.BTN_A
			a.led = allLEDS
			train = False
			save =  ''
		elif(buttons & cwiid.BTN_1):
			a.led = 0
			train = True
		elif(buttons & cwiid.BTN_2):
			a.led = cwiid.LED1_ON ^ cwiid.LED4_ON
			save = 'load'
		elif(buttons & cwiid.BTN_MINUS):
			if (train):
				destroy = True
		elif(buttons & cwiid.BTN_PLUS):
			if (save == 'load'):
				save = 'save'
		elif (buttons & cwiid.BTN_LEFT):
			mode = cwiid.BTN_LEFT
			a.led = cwiid.LED1_ON
		elif (buttons & cwiid.BTN_UP):
			mode = cwiid.BTN_UP
			a.led = cwiid.LED2_ON
		elif (buttons & cwiid.BTN_RIGHT):
			mode = cwiid.BTN_RIGHT
			a.led = cwiid.LED3_ON
		elif (len(obs) != 0):
			if (mode == cwiid.BTN_A):
				sample = []
				for i in xrange(0,30):
					j = (len(obs)-1)*1./30.*i
					j = int(j)
					sample.append(obs[j])
				dis = {cos(models[0],sample):leftAction, cos(models[2],sample):rightAction, cos(models[1],sample):forwardAction}
				if (min(dis.keys()) < 100):
					print dis[min(dis.keys())](),min(dis.keys())
				else:
					print min(dis.keys())

			else:
				if (mode == cwiid.BTN_LEFT):
					mdl = 0
				elif(mode == cwiid.BTN_UP):
					mdl = 1
				elif(mode == cwiid.BTN_RIGHT):
					mdl = 2
				else:
					mdl = 1

				if (save == 'save'):
					print "saving %s..." % (mdl,)
					try:
						file = open("%s.mdl" % (mdl,),'w')
						dump(models[mdl],file)
						file.close()
						a.rumble = 1
						sleep(.5)
						a.rumble = 0
					except:
						print "problem saving"
					save = ''
					a.led = 0
				elif (save == 'load'):
					print "loading %s..." % (mdl,)
					try:
						file = open("%s.mdl" % (mdl,),'r')
						models[mdl] = load(file)
						file.close()
						a.rumble = 1
						sleep(.5)
						a.rumble = 0
					except:
						print "problem loading"
					save = ''
					a.led = 0
				elif (destroy):
					counts[mdl] = 0
					models[mdl] = []
					for j in xrange(0,30):
						models[mdl].append([0,0,0])
					destroy = False
				else:
					counts[mdl] = counts[mdl] + 1
					for i in xrange(0,30):
						j = (len(obs)-1)*1./30.*i
						j = int(j)
						models[mdl][i][0] = (models[mdl][i][0]*float(counts[mdl]-1) + obs[j][0]) / float(counts[mdl])
						models[mdl][i][1] = (models[mdl][i][1]*float(counts[mdl]-1) + obs[j][1]) / float(counts[mdl])
						models[mdl][i][2] = (models[mdl][i][2]*float(counts[mdl]-1) + obs[j][2]) / float(counts[mdl])
			obs = []

except Exception as e:
	print e
	pass

finally:
	pass
	create.stop()
