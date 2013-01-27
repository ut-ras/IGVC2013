#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
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

tests = [\
  {"vector": [[1], [1], [0]],\
   "thetas":[0, 0, math.pi/4]},\
  {"vector": [[0], [1], [1]],\
   "thetas":[math.pi/4, 0, 0]},\
  {"vector": [[1], [0], [1]],\
   "thetas":[0, math.pi/4, 0]},\
  {"vector": [[1], [1], [1]],\
   "thetas":[math.pi/4, 0, math.pi/4]},\
]

for test in tests:
  vector = test["vector"]
  thetas = test["thetas"]
  print("")

def um6_imu_callback(data):
    rospy.loginfo(data)

    thetas = [data.orientation_euler.pitch, data.orientation_euler.yaw, data.orientation_euler.roll]
    vector = [[data.accelerometer.x],[data.accelerometer.y],[data.accelerometer.z]]

    for i in range(0, 3):
        thetas[i] *= math.pi/180.0
    
    print(vector)
    print(thetas)
    print(rotate(vector, thetas))
    print("")
    

if __name__ == '__main__':
    rospy.init_node('imu_rotater_test', anonymous=False)

    rospy.Subscriber("um6_imu_data", UM6IMU, um6_imu_callback)

    rospy.spin()


