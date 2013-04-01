#!/usr/bin/env python
#
#driver for UM6-LT Orientation Sensor
#datasheet: pololu.com/file/0J442/UM6_datasheet.pdf
#Author: Cruz Monrreal II
#        Frank Weng
#        Sagar Tewari
#        Robby Nevels

import roslib; roslib.load_manifest('um6_imu')
import rospy
from um6_imu.msg import UM6IMU
import serial
import struct
import time


DO_SCALING = True
GYRO_SCALE = 0.0610352
ACCEL_SCALE = 0.000183105
MAG_SCALE = 0.000305176
EULER_SCALE = 0.0109863
QUAT_SCALE = 0.0000335693

ser = serial.Serial(port = '/dev/UM6-LT', baudrate=115200)

def um6():
    rospy.init_node('um6_imu')
    rospy.loginfo("UM6-LT IMU listener is running on " + ser.portstr )

    ser.flush()
    ser.open()

    while not rospy.is_shutdown():
        pub = rospy.Publisher('um6_imu_data', UM6IMU)
        p = UM6IMU()
        data = ""
        while not rospy.is_shutdown(): # should not run forever
            # Read bytes
            while ser.inWaiting() > 0:
                data += ser.read(1)

            while data.__len__() >= 8:
                if data.startswith("snp"):
                    # Addr + BL*2 + Chk1 + Chk0
                    remaining = 1 + 4*((int(data[3].encode('hex'),16) & 0x3C) >> 2) + 1 + 1
                    datanum = (int(data[3].encode('hex'),16) & 0x3C) >> 2

                    # Wait until the rest of the bytes are ready
                    while data.__len__() < 8+remaining:
                        # Read bytes
                        while ser.inWaiting() > 0:
                            data+=ser.read(1)

                    if data[4].encode('hex') == "5c":
                        gyro_yz = int(data[5:7].encode('hex'),16)
                        gyro_yz -= 0x10000 if gyro_yz & 0x8000 else 0x0
                        #print "Gyro YZ:", gyro_yz*0.0610352

                        gyro_xz = int(data[7:9].encode('hex'),16)
                        gyro_xz -= 0x10000 if gyro_xz & 0x8000 else 0x0
                        #print "Gyro XZ:", gyro_xz*0.0610352

                        gyro_xy = int(data[9:11].encode('hex'),16)
                        gyro_xy -= 0x10000 if gyro_xy & 0x8000 else 0x0
                        #print "Gyro XY:", gyro_xy*0.0610352

                        if DO_SCALING:
                            gyro_yz *= GYRO_SCALE
                            gyro_xz *= GYRO_SCALE
                            gyro_xy *= GYRO_SCALE

                        p.gyro.x = gyro_yz
                        p.gyro.y = gyro_xz
                        p.gyro.z = gyro_xy

                    elif data[4].encode('hex') == "5e":
                        accel_x = int(data[5:7].encode('hex'),16)
                        accel_x -= 0x10000 if accel_x & 0x8000 else 0x0
                        accel_x = int(accel_x)
                        #print "Accel X:", accel_x*0.000183105

                        accel_y = int(data[7:9].encode('hex'),16)
                        accel_y -= 0x10000 if accel_y & 0x8000 else 0x0
                        #print "Accel Y:", accel_y*0.000183105

                        accel_z = int(data[9:11].encode('hex'),16)
                        accel_z -= 0x10000 if accel_z & 0x8000 else 0x0
                        #print "Accel Z:", accel_z*0.000183105

                        if DO_SCALING:
                            accel_x *= ACCEL_SCALE
                            accel_y *= ACCEL_SCALE
                            accel_z *= ACCEL_SCALE

                        p.accelerometer.x = accel_x
                        p.accelerometer.y = accel_y
                        p.accelerometer.z = accel_z

                    elif data[4].encode('hex') == "60":
                        mag_x = int(data[5:7].encode('hex'),16)
                        mag_x -= 0x10000 if mag_x & 0x8000 else 0x0
                        #print "Mag X:", mag_x*0.000305176

                        mag_y = int(data[7:9].encode('hex'),16)
                        mag_y -= 0x10000 if mag_y & 0x8000 else 0x0
                        #print "Mag Y:", mag_y*0.000305176

                        mag_z = int(data[9:11].encode('hex'),16)
                        mag_z -= 0x10000 if mag_z & 0x8000 else 0x0
                        #print "Mag Z:", mag_z*0.000305176

                        if DO_SCALING:
                            mag_x *= MAG_SCALE
                            mag_y *= MAG_SCALE
                            mag_z *= MAG_SCALE

                        p.compass.x = mag_x
                        p.compass.y = mag_y
                        p.compass.z = mag_z

                    elif data[4].encode('hex') == "62":
                        roll = int(data[5:7].encode('hex'),16)
                        roll -= 0x10000 if roll & 0x8000 else 0x0
                        #print "Roll:", roll*0.0109863

                        pitch = int(data[7:9].encode('hex'),16)
                        pitch -= 0x10000 if pitch & 0x8000 else 0x0
                        #print "Pitch:", pitch*0.0109863

                        yaw = int(data[9:11].encode('hex'),16)
                        yaw -= 0x10000 if yaw & 0x8000 else 0x0
                        #print "Yaw:", yaw*0.0109863

                        if DO_SCALING:
                            roll *= EULER_SCALE
                            pitch *= EULER_SCALE
                            yaw *= EULER_SCALE

                        p.orientation_euler.roll = roll
                        p.orientation_euler.pitch = pitch
                        p.orientation_euler.yaw = yaw

                    elif data[4].encode('hex') == "64":
                        quat_a = int(data[5:7].encode('hex'),16)
                        quat_a -= 0x10000 if quat_a & 0x8000 else 0x0
                        #print "Quat A:", quat_a*0.0000335693

                        quat_b = int(data[7:9].encode('hex'),16)
                        quat_b -= 0x10000 if quat_b & 0x8000 else 0x0
                        #print "Quat B:", quat_b*0.0000335693

                        quat_c = int(data[9:11].encode('hex'),16)
                        quat_c -= 0x10000 if quat_c & 0x8000 else 0x0
                        #print "Quat C:", quat_c*0.0000335693

                        quat_d = int(data[11:13].encode('hex'),16)
                        quat_d -= 0x10000 if quat_d & 0x8000 else 0x0
                        #print "Quat D:", quat_d*0.0000335693

                        if DO_SCALING:
                            quat_a *= QUAT_SCALE
                            quat_b *= QUAT_SCALE
                            quat_c *= QUAT_SCALE
                            quat_d *= QUAT_SCALE

                        p.orientation_quat.x = quat_a
                        p.orientation_quat.y = quat_b
                        p.orientation_quat.z = quat_c
                        p.orientation_quat.w = quat_d

                        p.header.stamp = rospy.get_rostime()

                        pub.publish(p)

                    else:
                        print data[4].encode('hex')

                    # Remove bytes from buffer
                    data = data[4+remaining:]
                else:
                    #print "Data starts with: ", data[0:2]
                    data = data[1:]   # Shift by one


if __name__ == "__main__":
  try:
    um6()
  except rospy.ROSInterruptException: pass

