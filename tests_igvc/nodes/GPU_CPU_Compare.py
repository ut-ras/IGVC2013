#!/usr/bin/env python
import roslib; roslib.load_manifest('tests_igvc')
import rospy

from Graphics import Graphics
from std_msgs.msg import UInt32

class Data:
    def __init__(self, name):
        self.name = name
        self.total = 0.0
        self.total_sq = 0.0
        self.count = 0.0

    def update(self, data):
        self.total += data
        self.total_sq += data**2
        self.count += 1.0

    def disp(self):
        ave = self.total/self.count
        var = self.total_sq/self.count - ave**2
        print self.total_sq/self.count, ave**2
        print self.name + ":", ave, var

gpu_data = Data("GPU")
cpu_data = Data("CPU")

def gpu_callback (data):
    global gpu_data
    gpu_data.update(data.data/1000000.0)
    gpu_data.disp()

def cpu_callback(data):
    global cpu_data
    cpu_data.update(data.data/1000000.0)
    cpu_data.disp()

if __name__ == '__main__':
    rospy.init_node('GPU_CPU_Compare')

    rospy.Subscriber("/gpu_profile", UInt32, gpu_callback)
    rospy.Subscriber("/cpu_profile", UInt32, cpu_callback)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        print "hi"
        r.sleep()
