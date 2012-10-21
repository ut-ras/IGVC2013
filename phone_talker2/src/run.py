#!/usr/bin/env python
import roslib; roslib.load_manifest('phone_talker')
import rospy
from geometry_msgs.msg import Twist

import twilio.twiml
from flask import Flask

app = Flask(__name__)
pub = rospy.Publisher('vel_cmd',Twist)
rospy.init_node('phone_talker')
@app.route("/", methods=['GET', 'POST'])
def granny_control():
    """Respond to incoming requests."""
    resp = twilio.twiml.Response()
    resp.say("Welcome to the Granny control system.")
    
    return redirect("/getKey")

@app.route("/getKey", methods=['GET', 'POST'])
def get_key():
    """Waiting for a key press."""
    with resp.gather(numDigits=1, action="/handleKey", method="POST") as g:
        g.say("Use your keypad to move.")
    return str(resp)

@app.route("/handleKey", methods=['GET','POST'])
def handle_key():
    """Handles key press"""
    digit_pressed = request.values.get('Digits',None)
    if digit_pressed == "2":
        # move forward
        twst = Twist()
        twst.linear.x = 255
        rospy.loginfo("Moving forward. (Phone)")
        pub.publish(twst)
    if digit_pressed == "8":
        #move back
        twst = Twist()
        twst.linear.x = 0
        rospy.loginfo("Moving back. (Phone)")
        pub.publish(twst)
    if digit_pressed == "4":
        # rotate left
        twst = Twist()
        twst.angular.z = 0
        rospy.loginfo("Rotating left. (Phone)")
        pub.publish(twst)
    if digit_pressed == "6":
        # rotate right
        twst = Twist()
        twst.angular.z = 255
        rospy.loginfo("Rotating right. (Phone)")
        pub.publish(twst)
    return redirect("/getKey")


if __name__ == "__main__":
    try:
        app.run(debug=True)
    except rospy.ROSInterruptException: pass

