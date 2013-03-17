#!/usr/bin/env python
#
#driver for VN-200 IMU
#datasheet:
#Author: Cruz Monrreal II

import roslib; roslib.load_manifest('vn_200_imu')
import rospy
from vn_200_imu.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
import serial
import struct
import time


DO_SCALING = True

IMU_MSG_LEN = 12
GPS_SOLN_MSG_LEN = 16
INS_SOL_MSG_LEN = 16

ser = serial.Serial(port = '/dev/VN-200', baudrate=921600)

def read_data() :

    data = "$"

    while not rospy.is_shutdown():
        c = ser.read(1)
        if c == '$':
            break

    while not rospy.is_shutdown():
        c = ser.read(1)
        if c == '\n':
            break
        elif c == '\x00':
            continue
        data += c

    return data



def cmd(string):
    xor = 0

    for char in string.upper():
        xor = xor ^ ord(char)

    return '$' + string + '*' + hex(xor & 0xFF)[2:4].upper() + '\n'

CMD_TO_READ_NEXT = 0
READ_CMDS = [cmd("VNRRG,54"), \
             cmd("VNRRG,58"), \
             cmd("VNRRG,63")]


def vn200():

    global CMD_TO_READ_NEXT
    global READ_CMDS
    global GPS_SOLN_MSG_LEN
    global INS_SOL_MSG_LEN
    global IMU_MSG_LEN

    rospy.init_node('vn_200_imu')
    rospy.loginfo("VN-200 IMU listener is running on " + ser.portstr )

    ser.flush()
    ser.open()

    imu_pub = rospy.Publisher('vn_200_accel_gyro_compass', vn_200_accel_gyro_compass)
    gps_pub = rospy.Publisher('vn_200_gps_soln', vn_200_gps_soln)
    ins_pub = rospy.Publisher('vn_200_ins_soln', vn_200_ins_soln)


    ser.write(cmd("VNWRG,05,921600"))
    ser.write(cmd("VNWRG,07,50"))
    ser.write(cmd("VNWRG,06,0"))

    while not rospy.is_shutdown():

        ser.write(READ_CMDS[CMD_TO_READ_NEXT])

        CMD_TO_READ_NEXT = (CMD_TO_READ_NEXT + 1) % len(READ_CMDS)

        data = read_data()

        if data[7:9] == "54" and data[1:6] == "VNRRG":

            imu_msg = vn_200_accel_gyro_compass()
            imu_data = data[7:-4].split(',')

            if len(imu_data) is not IMU_MSG_LEN:
                continue

            rospy.loginfo("IMU: " + str(imu_data))

            imu_msg.header.stamp = rospy.get_rostime()

            imu_msg.compass.x = float(imu_data[0])
            imu_msg.compass.y = float(imu_data[1])
            imu_msg.compass.z = float(imu_data[2])

            imu_msg.accelerometer.x = float(imu_data[3])
            imu_msg.accelerometer.y = float(imu_data[4])
            imu_msg.accelerometer.z = float(imu_data[5])

            imu_msg.gyro.x = float(imu_data[6])
            imu_msg.gyro.y = float(imu_data[7])
            imu_msg.gyro.z = float(imu_data[8])

            imu_pub.publish(imu_msg)


        elif data[7:9] == "58" and data[1:6] == "VNRRG":

            gps_msg = vn_200_gps_soln()
            gps_data = data[7:-4].split(',')

            if len(gps_data) is not GPS_SOLN_MSG_LEN:
                continue

            rospy.loginfo("GPS: " + str(gps_data))

            gps_msg.header.stamp = rospy.get_rostime()

            gps_msg.latitude = float(gps_data[4])
            gps_msg.longitude = float(gps_data[5])

            gps_msg.num_satellites = int(gps_data[3])

            gps_msg.ned_velocities.x = float(gps_data[7])
            gps_msg.ned_velocities.y = float(gps_data[8])
            gps_msg.ned_velocities.z = float(gps_data[9])

            gps_msg.ned_acceleration.x = float(gps_data[10])
            gps_msg.ned_acceleration.y = float(gps_data[11])
            gps_msg.ned_acceleration.z = float(gps_data[12])

            gps_msg.speed_accuracy_estimate = float(gps_data[13])

            gps_pub.publish(gps_msg)

        elif  data[7:9] == "63" and data[1:6] == "VNRRG":


            ins_msg = vn_200_ins_soln()
            ins_data = data[7:-4].split(',')

            if len(ins_data) is not INS_SOL_MSG_LEN:
                continue

            rospy.loginfo("INS: " + str(ins_data) + '\n')

            ins_msg.orientation_euler.yaw = float(ins_data[3])
            ins_msg.orientation_euler.pitch = float(ins_data[4])
            ins_msg.orientation_euler.roll = float(ins_data[5])

            ins_msg.geodetic_latitude = float(ins_data[6])
            ins_msg.geodetic_longitude = float(ins_data[7])
            ins_msg.altitude = float(ins_data[8])

            ins_msg.ned_velocities.x = float(ins_data[9])        # Velocity X
            ins_msg.ned_velocities.y = float(ins_data[10])       # Velocity Y
            ins_msg.ned_velocities.z = float(ins_data[11])       # Velocity Z

            ins_msg.attitude_uncertainty = float(ins_data[12])# Altitude Uncertainty
            ins_msg.position_uncertainty = float(ins_data[13])# Position Uncertainty
            ins_msg.velocity_uncertainty = float(ins_data[14])# Velocity Uncertainty

            ins_pub.publish(ins_msg)

        else:
            rospy.loginfo("Unknown message: " + str(data) + '\n')



if __name__ == "__main__":
    try:
      vn200()
    except rospy.ROSInterruptException: pass

