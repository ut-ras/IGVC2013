#!//usr/bin/env python

import roslib; roslib.load_manifest('GPS_Ublox')
import rospy
from GPS_Ublox.msg import GPS_UBlox_raw

output = open('GPS_Coords.txt', 'w')

def write_to_file(data):
    lat = str(data.latitude)
    lon = str(data.longitude)

    rospy.loginfo(lat + ', ' + lon + '\n')

    output.write(lat + ', ' + lon + '\n')


def listener():
    rospy.init_node('GPS_Coordinate_Listener', anonymous=False)
    rospy.Subscriber('gps_data_raw', GPS_UBlox_raw, write_to_file)
    rospy.spin()

if __name__ == '__main__':
    global output
    try:
        listener()
    except KeyboardInterrupt:
        global output
        output.close()
    except rospy.ROSInterruptException:
        global output
        output.close()

