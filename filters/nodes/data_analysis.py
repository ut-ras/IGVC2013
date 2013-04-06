#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, tf

from vn_200_imu.msg import vn_200_accel_gyro_compass

class SensorData:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.sum = 0
        self.squared_sum = 0

    def update(self, val):
        self.count += 1.0
        self.sum += val
        self.squared_sum += val**2

        self.mean = self.sum/self.count
        self.var = self.squared_sum/self.count - self.mean**2

    def dispData(self):
        print self.name, "mean:", self.mean, "var:", self.var

vn_200_data_accelx = SensorData("accelx");
vn_200_data_accely = SensorData("accely");
vn_200_data_accelz = SensorData("accelz");

def vn_200_imu_callback(data):
    vn_200_data_accelx.update(data.accelerometer.x)
    vn_200_data_accelx.dispData()
    vn_200_data_accely.update(data.accelerometer.y)
    vn_200_data_accely.dispData()
    vn_200_data_accelz.update(data.accelerometer.z)
    vn_200_data_accelz.dispData()


if __name__ == '__main__':
    rospy.init_node('data_analysis', anonymous=True)

    rospy.Subscriber("vn_200_accel_gyro_compass", vn_200_accel_gyro_compass, vn_200_imu_callback)

    rospy.spin()
