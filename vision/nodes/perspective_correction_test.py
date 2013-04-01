#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# for displaying the windows conveniently
EXPECTED_WIDTH = 640
WINDOW_HEIGHT = 30

# using previous transform or calculate a new one
CALC_TRANSFORM = False
prev_transform = np.array(
[[  2.48253566e+00,   1.65852542e+00,  -5.01512515e+02],
 [  8.35395261e-03,   4.60948006e+00,  -6.44248503e+02],
 [  6.77126073e-05,   4.81987961e-03,   1.00000000e+00]])

"""
[[ -3.38211005e+00,  -3.63700168e+00,   7.24379943e+02],
 [ -4.58408261e-01,  -7.79335168e+00,   9.49241798e+02],
 [ -2.06572848e-03,  -2.00950642e-02,   1.00000000e+00]])
"""

CALC_PERIOD = 1 # in seconds

# (cols, rows) of expected interior corners (so an 8x8 chessboard would have (7, 7) here)
boardSize = (7, 7)

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()


###
# Begin borrowed code from online guide
###

def getOuterPoints(rcCorners):
   tl = rcCorners[0,0]
   tr = rcCorners[0,-1]
   bl = rcCorners[-1,0]
   br = rcCorners[-1,-1]
   if tl[0] > tr[0]:
      tr,tl = tl,tr
      br,bl = bl,br
   if tl[1] > bl[1]:
      bl,tl=tl,bl
      br,tr=tr,br
   return (tl,tr,bl,br)

def calcTransform(img, corners, boardSize):
    size = img.shape[1],img.shape[0]

    rcCorners = corners.reshape(boardSize[0], boardSize[1], 2)

    outerPoints = getOuterPoints(rcCorners)
    tl,tr,bl,br = outerPoints

    patternSize = np.array([
        np.sqrt(((tr - tl)**2).sum(0)),
        np.sqrt(((bl - tl)**2).sum(0)),
    ])

    inQuad = np.array(outerPoints, np.float32)

    outQuad = np.array([
        tl,
        tl + np.array([patternSize[0],0.0]),
        tl + np.array([0.0,patternSize[1]]),
        tl + patternSize,
    ],np.float32)

    return cv2.getPerspectiveTransform(inQuad, outQuad)

###
# end borrowed code
###

def dispImages(img):
    if CALC_TRANSFORM:
        (found, corners) = cv2.findChessboardCorners(
            img,
            boardSize,
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | \
                    cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not found:
            rospy.loginfo("ERROR: cannot find chess board in image")
            return;

        transform = calcTransform(img, corners, boardSize)
        print transform
        transformed = cv2.warpPerspective(img, transform, (img.shape[1],img.shape[0]))

        cv2.drawChessboardCorners(img, boardSize, corners, found)
    else:
        global prev_transform
        transformed = cv2.warpPerspective(img, prev_transform, (img.shape[1], img.shape[0]))

    cv2.imshow('Transformed', transformed)
    cv2.imshow('Plain', img)
    cv2.imwrite('plain.png', img)
    cv2.imwrite('transformed.png', transformed)

scale = .5
rot_mat = cv2.getRotationMatrix2D((0,0), 0, scale)

def scaleDown(img):
    rows,cols = img.shape[:2]
    scaled_img = cv2.warpAffine(img, rot_mat, (int(cols*scale), int(rows*scale)))
    return scaled_img

curImg = None
prev_time = -1
def callback(image_data):
    if CALC_TRANSFORM:
        global prev_time
        cur_time = rospy.get_time()
        if cur_time - prev_time < CALC_PERIOD:
            return
        prev_time = cur_time

    try:
        input_img = np.asarray(bridge.imgmsg_to_cv(image_data, "bgr8"))
    except CvBridgeError, e:
        print e

    rospy.loginfo("received image")

    global curImg
    #curImg = scaleDown(input_img)
    curImg = input_img

def init():
    rospy.init_node('perspective_correction_test')
    prev_time = rospy.get_time()

    cv2.namedWindow('Plain')
    cv2.moveWindow('Plain', 0, WINDOW_HEIGHT)

    cv2.namedWindow('Transformed')
    cv2.moveWindow('Transformed', EXPECTED_WIDTH, WINDOW_HEIGHT)

    rospy.loginfo("Now, move the chessboard into view to get a correct perspective transformation!")

    sub = rospy.Subscriber('usb_cam/image_raw', Image, callback )

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        global curImg
        if curImg != None:
            rospy.loginfo("processing image")
            dispImages(curImg)
            cv2.waitKey(30)

        r.sleep()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass

    cv2.destroyAllWindows() # not sure if this does anything































