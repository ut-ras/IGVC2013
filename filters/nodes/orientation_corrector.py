#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, math

from vn_200_imu.msg import vn_200_ins_soln, vn_200_accel_gyro_compass
from filters.msg import Orientation

YAW_CORRECTION = -math.pi/2
"""
X_OFFSET = 2.45331919332
Y_OFFSET = 1.564559527120
Z_OFFSET = 2.0
"""
X_OFFSET = 0.0
Y_OFFSET = 0.0
Z_OFFSET = 0.0
pub = None

def bound0to2Pi(angle):
    return (angle%(2*math.pi) + 2*math.pi)%(2*math.pi)

def vn_200_ins_callback(data):
    roll = bound0to2Pi(data.orientation_euler.roll*math.pi/180.0)
    pitch = bound0to2Pi(data.orientation_euler.pitch*math.pi/180.0)
    yaw = -data.orientation_euler.yaw*math.pi/180.0

    yaw = bound0to2Pi(yaw + YAW_CORRECTION)
    yaw = math.pi*2 - yaw

    msg = Orientation()
    msg.roll = roll
    msg.pitch = pitch
    msg.yaw = yaw

    global pub
    pub.publish(msg)

alphaMag = 0.2
alphaAccel = 0.2

xMagOld = 0.0
yMagOld = 0.0
zMagOld = 0.0

xAccelOld = 0.0
yAccelOld = 0.0
zAccelOld = 0.0

def vn_200_imu_callback(data):
    # read compass data
    global X_OFFSET, Y_OFFSET, Z_OFFSET
    xMag = data.compass.x - X_OFFSET
    yMag = data.compass.y - Y_OFFSET
    zMag = data.compass.z - Z_OFFSET

    # mag low-pass filter
    global alphaMag, xMagOld, yMagOld, zMagOld
    xMag = xMag*alphaMag + xMagOld*(1 - alphaMag)
    yMag = yMag*alphaMag + yMagOld*(1 - alphaMag)
    zMag = zMag*alphaMag + zMagOld*(1 - alphaMag)

    xMagOld = xMag
    yMagOld = yMag
    zMagOld = zMag

    # normalize the mag values
    norm = math.sqrt(xMag**2 + yMag**2 + zMag**2)
    xMag /= norm
    yMag /= norm
    zMag /= norm

    # read accel data
    xAccel = data.accelerometer.x
    yAccel = data.accelerometer.y
    zAccel = data.accelerometer.z

    # accel low-pass filter
    global alphaAccel, xAccelOld, yAccelOld, zAccelOld
    xAccel = xAccel*alphaAccel + xAccelOld*(1 - alphaAccel)
    yAccel = yAccel*alphaAccel + yAccelOld*(1 - alphaAccel)
    zAccel = zAccel*alphaAccel + zAccelOld*(1 - alphaAccel)

    xAccelOld = xAccel
    yAccelOld = yAccel
    zAccelOld = zAccel

    # normalize the mag values
    norm = math.sqrt(xAccel**2 + yAccel**2 + zAccel**2)
    xAccel /= norm
    yAccel /= norm
    zAccel /= norm


    # calculate roll & pitch, then yaw
    # src: http://arduino.cc/forum/index.php/topic,8573.0.html
    roll = math.atan2(yAccel, math.sqrt(xAccel**2 + zAccel**2))
    pitch = math.atan2(xAccel, math.sqrt(yAccel**2 + zAccel**2))
    yaw = math.atan2(
            -yMag*math.cos(roll) + zMag*math.sin(roll),
            xMag*math.cos(pitch) + zMag*math.sin(pitch)*math.sin(roll)
                + zMag*math.sin(pitch)*math.cos(roll)
            )

    msg = Orientation()
    msg.roll = bound0to2Pi(roll)
    msg.pitch = bound0to2Pi(pitch)
    msg.yaw = bound0to2Pi(math.pi*2 - yaw + YAW_CORRECTION)

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
    """
    rospy.Subscriber(
            "vn_200_ins_soln",
            vn_200_ins_soln,
            vn_200_ins_callback
            )
    """

    rospy.loginfo("subscribed to ins messages!")
    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass

