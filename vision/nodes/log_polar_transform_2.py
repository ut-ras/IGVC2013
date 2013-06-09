#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv, math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# set this to true to show images being transformed
DISPLAY_STUFF = False

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()

# these transforms are determined from using a chess board and the magic
# code in perspective_correction_test.py (with its CALC_TRANSFORM set to true)
# note that these are only correctly defined for a particular image resolution
# (at the time of writing, the resolution of incoming images is 640x480
frameTransformMap = {
	'cam_center': np.array(
		[[  1.78778569e+00,   9.19775038e-01,  -2.56236007e+02],
 		 [  2.24321245e-02,   2.98569665e+00,  -3.37687301e+02],
 		 [  8.04901494e-05,   2.72273026e-03,   1.00000000e+00]]),
	'cam_left': np.array(
		[[  1.78778569e+00,   9.19775038e-01,  -2.56236007e+02],
 		 [  2.24321245e-02,   2.98569665e+00,  -3.37687301e+02],
 		 [  8.04901494e-05,   2.72273026e-03,   1.00000000e+00]]),
	'cam_right': np.array(
		[[  1.78778569e+00,   9.19775038e-01,  -2.56236007e+02],
 		 [  2.24321245e-02,   2.98569665e+00,  -3.37687301e+02],
 		 [  8.04901494e-05,   2.72273026e-03,   1.00000000e+00]])
	}

"""
TODO: THESE TWO CONSTANTS SHOULD BE IN THE PARAMETER SERVER; THEY WILL BE
USED IN WHATEVER NODE PROCESSES THE LOGPOLAR TOPIC
"""
logscale = 20
anglescale = 80
	
pub = None

def homography_transform(img, transform):
    return cv2.warpPerspective(
            img,
            transform,
            (img.shape[1], img.shape[0])
            )

def log_polar_transform(img):
    center = (img.width/2, img.height)
    maxradius = math.sqrt((img.width/2)**2 + (img.height)**2)
    width = int(logscale*math.log(maxradius))

    logpolar = cv.CreateImage(
            (width, anglescale),
            cv.IPL_DEPTH_8U,
            3 # exercise: why is does this need to be 3 here, but 1 in polar_processor.py? (btw I have no idea)
            )

    cv.LogPolar(
            img,
            logpolar,
            center,
            logscale,
            cv.CV_INTER_LINEAR + cv.CV_WARP_FILL_OUTLIERS
            )

    aligned = cv.GetSubRect(logpolar, (0, anglescale/2, width, anglescale/2))

    return aligned

initWindowFrameIdMap = {
	'cam_center': False,
	'cam_left': False,
	'cam_right': False
	}

def display(homography, log_polar, frame_id):
    global initWindowFrameIdMap
    if not initWindowFrameIdMap[frame_id]:
        cv2.namedWindow('homography_'+frame_id)
        cv.NamedWindow('log_polar_'+frame_id)
        initWindowFrameIdMap[frame_id] = True

    cv2.imshow('homography_'+frame_id, homography)
    cv.ShowImage('log_polar_'+frame_id, log_polar)
    
    cv2.waitKey(30)

def handleImage(image, frame_id):
	cv2img = np.asarray(image)
	
	if not (frame_id in frameTransformMap.keys()):
		rospy.logwarn('recieved unknown frame_id: %s', frame_id)
		return;
	
	homography = homography_transform(cv2img, frameTransformMap[frame_id])
	cvimg_homography = cv.fromarray(homography)
	log_polar = log_polar_transform(cvimg_homography)

	msg = bridge.cv_to_imgmsg(log_polar, encoding="passthrough")
	msg.header.frame_id = frame_id
	
	global pub
	pub.publish(msg)
	
	if DISPLAY_STUFF:
		display(homography, log_polar, frame_id)

def callback(image_data):
    try:
        input_img = bridge.imgmsg_to_cv(image_data, "bgr8")
        handleImage(input_img, image_data.header.frame_id)
    except CvBridgeError, e:
    	print e
        rospy.logwarn('error processing frame')

def init():
    rospy.init_node('log_polar_transformer_2')

    try:
        subtopic = str(rospy.get_param('~subtopic'))
    except KeyError:
        rospy.logerr("yo we need the topic name broski: subtopic")
        return

    rospy.loginfo("listening to " + subtopic)

    global pub
    pub = rospy.Publisher('log_polar_transformed', Image)
    rospy.Subscriber(subtopic, Image, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































