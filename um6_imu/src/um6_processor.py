#!/usr/bin/env python
import roslib; roslib.load_manifest('um6_imu')
import rospy

import math

from um6_imu.msg import UM6IMU


def zero(m,n):
    # Create zero matrix
    new_matrix = [[0 for row in range(n)] for col in range(m)]
    return new_matrix

def mult(matrix1,matrix2):
    # Matrix multiplication
    if len(matrix1[0]) != len(matrix2):
        # Check matrix dimensions
        print 'Matrices must be m*n and n*p to multiply!'
    else:
        # Multiply if correct dimensions
        new_matrix = zero(len(matrix1),len(matrix2[0]))
        for i in range(len(matrix1)):
            for j in range(len(matrix2[0])):
                for k in range(len(matrix2)):
                    new_matrix[i][j] += matrix1[i][k]*matrix2[k][j]
        return new_matrix

def z_rot_matrix(theta):
  return [[math.cos(theta), math.sin(theta), 0.],\
          [-math.sin(theta), math.cos(theta), 0.],\
          [0., 0., 1.]]

def y_rot_matrix(theta):
  return [[math.cos(theta), 0., -math.sin(theta)],\
          [0., 1, 0.],\
          [math.sin(theta), 0, math.cos(theta)]]

def x_rot_matrix(theta):
  return [[1., 0., 0.],\
          [0., math.cos(theta), math.sin(theta)],\
          [0., -math.sin(theta), math.cos(theta)]]

def rotate(vector, thetas):
  theta_x = thetas[0]
  theta_y = thetas[1]
  theta_z = thetas[2]
  rot_matrix = mult(z_rot_matrix(theta_z), \
                mult(y_rot_matrix(theta_y), x_rot_matrix(theta_x)))
  return mult(rot_matrix, vector)


def um6_imu_callback(data):
  roll = data.orientation_euler.roll
  pitch = data.orientation_euler.pitch
  yaw = data.orientation_euler.yaw
  orients = [roll, pitch, yaw]

  accel_x = data.accelerometer.x
  accel_y = data.accelerometer.y
  accel_z = data.accelerometer.z
  accels = [[accel_x], [accel_y], [accel_z]]

  res = rotate(accels, orients)

  new_accel_x = res[0][0]
  new_accel_y = res[1][0]
  new_accel_z = res[2][0]

  rospy.loginfo("\n%.3f %.3f %.3f\n%.3f %.3f %.3f", accel_x, accel_y, accel_z, new_accel_x, new_accel_y, new_accel_z)

if __name__ == '__main__':
    rospy.init_node('um6_imu_processor', anonymous=False)

    rospy.Subscriber("um6_imu_data", UM6IMU, um6_imu_callback)

    rospy.spin()



