#!/usr/bin/env python
import roslib; roslib.load_manifest('phone_talker')
import rospy
from geometry_msgs.msg import Twist

import twilio.twiml
from flask import Flask, request

import time

def saturate(inp):
    return int(inp)

print "This app is now running"
app = Flask(__name__)
pub = rospy.Publisher('vel_cmd',Twist)
@app.route("/", methods=['GET', 'POST'])
def granny_control():
    """Respond to incoming requests."""
    resp = twilio.twiml.Response()
    resp.say("Welcome to the Granny control system.")
    resp.redirect("/getKey")
    return str(resp)

@app.route("/getKey", methods=['GET', 'POST'])
def get_key():
    """Waiting for a key press."""
    resp = twilio.twiml.Response()
    resp.gather(numDigits=1, action="/handleKey", method="POST", timeout="60")
    return str(resp)

@app.route("/handleKey", methods=['GET','POST'])
def handle_key():
    """Handles key press"""
    digit_pressed = request.values.get('Digits',None)
    if digit_pressed == "2":
        # move forward
        rospy.loginfo("Moving forward. (Phone)")
        for i in range(5):
            twst = Twist()
            twst.linear.x = 127
            pub.publish(twst)
            time.sleep(.1)
    if digit_pressed == "8":
        #move back
        rospy.loginfo("Moving back. (Phone)")
        for i in range(5):
            twst = Twist()
            twst.linear.x = -127
            pub.publish(twst)
            time.sleep(.1)
    if digit_pressed == "4":
        # rotate left
        rospy.loginfo("Rotating left. (Phone)")
        for i in range(5):
            twst = Twist()
            twst.angular.z = -127
            pub.publish(twst)
            time.sleep(.1)
    if digit_pressed == "6":
        # rotate right
        rospy.loginfo("Rotating right. (Phone)")
        for i in range(5):
            twst = Twist()
            twst.angular.z = 127
            pub.publish(twst)
            time.sleep(.1)
    resp = twilio.twiml.Response()
    resp.redirect("/getKey")
    return str(resp)

@app.route("/android",methods=['GET'])
def android():
    x = 0
    y = 0
    if request.method == 'GET':
        x = request.values.get('x', 0)
        y = request.values.get('y', 0)
    if request.method == 'POST':
        x = request.form['x']
        y = request.form['y']
    rospy.loginfo("Android data x:"+x+" y:"+y)
    twst = Twist()
    twst.angular.z = saturate(x)
    twst.linear.x = saturate(y)
    pub.publish(twst)
    return str("GTFO")


if __name__ == "__main__":
    try:
        print "Starting ROS node"
        rospy.init_node('phone_talker')
        print "ROS node started!"
        print "Starting the app!!"
        app.run(debug=True)
    except rospy.ROSInterruptException: pass

