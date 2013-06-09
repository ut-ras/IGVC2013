#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, math
from filters.msg import PlanarData

pub = None

topicIndexFrameIdMap = {
	'cam_center': 0,
	'cam_left': 1,
	'cam_right': 2
	}
	
NUM_TOPICS = len(topicIndexFrameIdMap.keys())
inputPDataArray = [PlanarData() for i in range(NUM_TOPICS)]

class AngleInfo:
    def __init__(self, min, inc):
        self.min = min
        self.inc = inc
        
def publish_scans():
    #
    # initialize the data message
    #
    data = PlanarData()

    data.startAngle = min([pdata.startAngle for pdata in inputPDataArray])
    endAngle = max([(pdata.startAngle + pdata.angleRange) for pdata in inputPDataArray])
    data.angleRange = endAngle - data.startAngle
    
    #
    # merge the scans into one
    #
    angles = []
    ranges = []
    
    indexes = [0]*NUM_TOPICS # used to index into the each of the topics' data arrays
    
    # put these aside so we don't do redundant work in the loop
    indexItr = range(NUM_TOPICS) # used to iterate over the indexes and topic data
    lengthArr = [len(pdata.ranges) for pdata in inputPDataArray]
	lengthsSum = sum(lengthArr) 
	
	# loop as long as we haven't gotten to the end of all the data arrays 
	while sum(indexes) < lengthsSum:
        minAngle = float("inf")
        bestIndex = None
        bestRange = None

        for i in indexItr:
            index = indexes[i]

            if index < lengthArr[i]:
                angle = inputPDataArray[i].angles[index]

                if angle < minAngle:
                    minAngle = angle
                    bestIndex = i
                    bestRange = inputPDataArray[i].ranges[index]

        indexes[bestIndex] += 1
        angles.append(minAngle)
        ranges.append(bestRange)

    data.angles = angles
    data.ranges = ranges

	global pub
    pub.publish(data)

def image_scan_transformed_callback(data):
	frame_id = data.header.frame_id

	if not (frame_id in topicIndexFrameIdMap.keys()):
		rospy.logwarn('recieved unknown frame_id: %s', frame_id)
		return;
		
	inputPDataArray[topicIndexFrameIdMap[frame_id]] = data
	publish_scans()

def init_subscriptions():
    global pub
    pub = rospy.Publisher("planar_data", PlanarData)
	
	rospy.Subscriber(
		"image_scan_transformed",
		PlanarData,
		image_scan_transformed_callback
		)

if __name__ == "__main__":
    rospy.init_node('ScanCombiner')
    init_subscriptions()
    rospy.loginfo("we've subscribed to the scan topics!");
    rospy.spin()
