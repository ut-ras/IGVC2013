#!/usr/bin/env python
import roslib; roslib.load_manifest('pcl_split')
import rospy
from sensor_msgs.msg import *

SUBTOPIC = 'pointcloud2'
default_split_num = 10
default_sleep_time = 1

split_num = 10
sleep_time = 1

pointcloud = None
pub = None

def processMsg(msg):
  global pointcloud
  pointcloud = msg


def pubNewMsg():
  global split_num
  global pointcloud

  if pointcloud == None:
    return

  size = pointcloud.width * pointcloud.height
  split_size = int(size / split_num)

  print "============================================"
  print "size = " + str(size)
  print "split_size = " +str(split_size)
  print "split_num = " + str(split_num)
  print "============================================"

  for i in range(split_num-1):
    p = PointCloud2()
    p.header = pointcloud.header
    p.height = i
    p.width = split_size
    p.fields = pointcloud.fields
    p.is_bigendian = pointcloud.is_bigendian
    p.is_dense = pointcloud.is_dense
    p.point_step = pointcloud.point_step
    p.row_step = p.width * p.point_step

    start = i * split_size * p.point_step
    end = (i+1) * split_size * p.point_step
    p.data = pointcloud.data[start:end]

    print "p.height = " + str(p.height)
    print "p.width = " + str(p.width)
    pub.publish(p)
    rospy.sleep(1)

  # publish last piece
  last_size = size - ((split_size) * (split_num-1))

  if(last_size > 0):
    p = PointCloud2()
    p.header = pointcloud.header
    p.height = split_num-1 
    p.width = last_size
    p.fields = pointcloud.fields
    p.is_bigendian = pointcloud.is_bigendian
    p.is_dense = pointcloud.is_dense
    p.point_step = pointcloud.point_step
    p.row_step = p.width * p.point_step

    start = split_size * (split_num-1) * p.point_step
    p.data = pointcloud.data[start:]
    print "p.height = " + str(p.height)
    print "p.width = " + str(p.width)

    pub.publish(p)
    rospy.sleep(1)


def pcl_split():
  global SUBTOPIC
  global pub
  global split_num

  split_num = rospy.get_param('split_num',default_split_num)
  sleep_time = rospy.get_param('pcl_split_sleep',default_sleep_time)

  PUBTOPIC = SUBTOPIC + '_split'
  pub = rospy.Publisher(PUBTOPIC,PointCloud2)
  rospy.Subscriber(SUBTOPIC,PointCloud2,processMsg)
  rospy.init_node('pcl_split')

  print 'Node initialized'

  while not rospy.is_shutdown():
    pubNewMsg()


  rospy.spin()

if __name__ == '__main__':
  try:
    pcl_split()
  except rospy.ROSInterruptException: pass
