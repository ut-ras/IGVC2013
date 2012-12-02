#!/usr/bin/env python
import socket
import struct
import sys

import roslib; roslib.load_manifest('phone_listener')
import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

class Gps:
    sig = 'g'
    topic = rospy.Publisher('phone/gps', Point)

    def pub(self, data):
        parsed = struct.unpack('!ddd',data[1:])

        msg = Point()
        msg.x = parsed[0]
        msg.y = parsed[1]
        msg.z = parsed[2]
        self.topic.publish(msg)

class Mag:
    sig = 'm'
    topic = rospy.Publisher('phone/mag', Vector3)

    def pub(self, data):
        parsed = struct.unpack('!ddd',data[1:])

        msg = Vector3()
        msg.x = parsed[0]
        msg.y = parsed[1]
        msg.z = parsed[2]
        self.topic.publish(msg)

class Accel:
    sig = 'a'
    topic = rospy.Publisher('phone/accel', Vector3)

    def pub(self, data):
        parsed = struct.unpack('!ddd',data[1:])

        msg = Vector3()
        msg.x = parsed[0]
        msg.y = parsed[1]
        msg.z = parsed[2]
        self.topic.publish(msg)


def main(socktype):
    rospy.init_node('phone_listener')

    sen = {s.sig: s for s in [Gps(), Mag(), Accel()]}
    sock = socket.socket(socket.AF_INET, socktype)
    sock.bind(('',11111))

    rospy.loginfo("Phone Listener is running: " + str(sock.getsockname()))

    if socktype == socket.SOCK_STREAM:
        sock.listen(1)  # recieve a connection if in tcp mode
        temp = sock.accept()
        sock.close()

        sock = temp[0]

        rospy.loginfo("Phone has connected")

    while not rospy.is_shutdown():
        data = sock.recv(25)

        sen[data[0]].pub(data)



if __name__ == "__main__":
    socktype = socket.SOCK_STREAM if sys.argv.count('tcp') else \
                socket.SOCK_DGRAM
    try:
        main(socktype)
    except rospy.ROSInterruptException:
        pass

