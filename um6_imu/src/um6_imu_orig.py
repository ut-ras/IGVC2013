#!/usr/bin/env python
#driver for the UM6-LT Orientation Sensor
#datasheet: pololu.com/file/0J442/UM6_datasheet.pdf
#Author: Frank Weng (weng.frank@utexas.edu)
import roslib; roslib.load_manifest('um6_imu')
import rospy
from um6_imu.msg import um6_imu
import serial, struct

ser = serial.Serial(port = rospy.get_param("port",'/dev/OceanServerIMU'), baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS)

def um6():
  rospy.init_node('um6_imu')
  rospy.loginfo("UM6-LT IMU listener is running on " + ser.portstr )
  #pub = rospy.Publisher('um6_imu_data', um6_imu)
  ser.flush()
  while not rospy.is_shutdown():
    #p = um6_imu()
    while True:
      print ord(struct.unpack('>1c', ser.read(1))[0])
    while struct.unpack('>1c', ser.read(1))[0] is not 's':
      pass
    if struct.unpack('>1c', ser.read(1))[0] is 'n':
      if struct.unpack('>1c', ser.read(1))[0] is 'p':
        #ser.read(2)
        data = struct.pack('>6B',ser.read(6))
        print struct.unpack('>2B2h',data)
        #p.gyro.x = ((ord(data[0])<<8)+ord(data[1]))*0.0610352
        #p.gyro.y = ((ord(data[2])<<8)+ord(data[3]))*0.0610352
        #p.gyro.z = ((ord(data[4])<<8)+ord(data[5]))*0.0610352

    #pub.publish(p)

if __name__ == "__main__":
  try:
    um6()
  except rospy.ROSInterruptException: pass

