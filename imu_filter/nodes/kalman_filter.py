#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_filter')
import rospy

import pylab
import math
import random
import numpy

from std_msgs.msg import String
from ocean_server_imu.msg import RawData
from imu_filter.msg import FilteredIMUData,KalmanFilteredData

kf = None
prevTime = None
pub = None

def publish_data(belief):
    msg = KalmanFilteredData()

    msg.position.x = belief[0][0]
    msg.velocity.x = belief[1][0]
    msg.acceleration.x = belief[2][0]
    msg.position.y = belief[3][0]
    msg.velocity.y = belief[4][0]
    msg.acceleration.y = belief[5][0]

    pub.publish(msg)

def imu_callback(data):
    # rospy.loginfo("Measure:\n%f\n%f", data.acceleration.x, data.acceleration.y)

    dt = 0
    curTime = data.header.stamp.to_time()
    global prevTime
    if prevTime != None:
        dt = curTime - prevTime
    prevTime = curTime

    control_vector = numpy.matrix([[0]])
    measurement_vector = numpy.matrix( [[data.acceleration.x],[data.acceleration.y]] )
    kf.Step(control_vector, measurement_vector, dt)
    
    belief = kf.GetCurrentState();
    rospy.loginfo("Belief:\n"+str(belief));
    publish_data(belief)

class KalmanFilterLinear:
  def __init__(self, _B, _H, _x, _P, _Q, _R):
    self.B = _B                      # Control matrix.
    self.H = _H                      # Observation matrix.
    self.current_state_estimate = _x # Initial state estimate.
    self.current_prob_estimate = _P  # Initial covariance estimate.
    self.Q = _Q                      # Estimated error in process.
    self.R = _R                      # Estimated error in measurements.

  def GetCurrentState(self):
    return self.current_state_estimate

  def Step(self, control_vector, measurement_vector, dt):
    self.A = numpy.matrix( [[1, dt, .5*dt*dt, 0, 0, 0], \
                            [0, 1, dt, 0, 0, 0],        \
                            [0, 0, 1, 0, 0, 0],         \
                            [0, 0, 0, 1, dt, .5*dt*dt], \
                            [0, 0, 0, 0, 1, dt],        \
                            [0, 0, 0, 0, 0, 1]] )
    #---------------------------Prediction step-----------------------------
    predicted_state_estimate = self.A * self.current_state_estimate #+ self.B * control_vector
    predicted_prob_estimate = (self.A * self.current_prob_estimate) * numpy.transpose(self.A) + self.Q
    #--------------------------Observation step-----------------------------
    innovation = measurement_vector - self.H*predicted_state_estimate
    innovation_covariance = self.H*predicted_prob_estimate*numpy.transpose(self.H) + self.R
    #-----------------------------Update step-------------------------------
    kalman_gain = predicted_prob_estimate * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
    self.current_state_estimate = predicted_state_estimate + kalman_gain * innovation
    # We need the size of the matrix so we can make an identity matrix.
    size = self.current_prob_estimate.shape[0]
    # eye(n) = nxn identity matrix.
    self.current_prob_estimate = (numpy.eye(size)-kalman_gain*self.H)*predicted_prob_estimate

def create_kalman_filter():
    control_matrix = numpy.matrix([[0]])
    observation_matrix = numpy.matrix([[0,0,1,0,0,0], [0,0,0,0,0,1]]);
    initial_state = numpy.matrix([[0],[0],[0],[0],[0],[0]])
    initial_probability = numpy.eye(6)
    process_covariance = numpy.eye(6)*1e-3
    measurement_covariance = numpy.eye(2)*0.03

    return KalmanFilterLinear(control_matrix,      \
                              observation_matrix,  \
                              initial_state,       \
                              initial_probability, \
                              process_covariance,  \
                              measurement_covariance)

if __name__ == '__main__':
    rospy.init_node('kalman_filter', anonymous=True)

    kf = create_kalman_filter()

    rospy.Subscriber("imu_filtered_data", FilteredIMUData, imu_callback)
    pub = rospy.Publisher('kalman_filtered_data', KalmanFilteredData)

    rospy.spin()


