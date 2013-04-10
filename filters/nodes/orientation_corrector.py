#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, math

from vn_200_imu.msg import vn_200_ins_soln, vn_200_accel_gyro_compass
from filters.msg import Orientation

YAW_CORRECTION = -math.pi/3.0

pub = None

def bound0to2Pi(angle):
    return (angle%(2*math.pi) + 2*math.pi)%(2*math.pi)

def vn_200_ins_callback(data):
    roll = bound0to2Pi(data.orientation_euler.roll*math.pi/180.0)
    pitch = bound0to2Pi(data.orientation_euler.pitch*math.pi/180.0)
    yaw = -data.orientation_euler.yaw*math.pi/180.0

    yaw = bound0to2Pi(yaw + YAW_CORRECTION)

    msg = Orientation()
    msg.roll = roll
    msg.pitch = pitch
    msg.yaw = yaw

    global pub
    pub.publish(msg)

def vn_200_imu_callback(data):
    xMag = data.compass.x
    yMag = data.compass.y
    zMag = data.compass.z

    # normalize the mag values
    norm = math.sqrt(xMag**2 + yMag**2 + zMag**2)
    xMag /= norm
    yMag /= norm
    zMag /= norm

    xAccel = data.accelerometer.x
    yAccel = data.accelerometer.y
    zAccel = data.accelerometer.z

    # normalize the accel vector
    norm = math.sqrt(xAccel**2 + yAccel**2 + zAccel**2)
    xAccel /= norm
    yAccel /= norm
    zAccel /= norm

    # calculate roll & pitch, then yaw
    # src: http://arduino.cc/forum/index.php/topic,8573.0.html
    roll = math.atan2(xAccel, math.sqrt(xAccel**2 + zAccel**2))  
    pitch = math.atan2(xAccel, math.sqrt(yAccel**2 + zAccel**2))
    yaw = math.atan2( 
            -yMag*math.cos(roll) + zMag*math.sin(roll), 
            xMag*math.cos(pitch) + zMag*math.sin(pitch)*math.sin(roll) + zMag*math.sin(pitch)*math.cos(roll)
            )

    msg = Orientation()
    msg.roll = roll
    msg.pitch = pitch
    msg.yaw = yaw

    global pub
    pub.publish(msg)

def init():
    rospy.init_node('orientation_corrector')

    global pub
    pub = rospy.Publisher('orientation_data', Orientation)
    rospy.Subscriber(
            "vn_200_accel_gyro_compass",
            vn_200_accel_gyro_compass,
            vn_200_imu_callback
            )

    rospy.loginfo("subscribed to ins messages!")
    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass

