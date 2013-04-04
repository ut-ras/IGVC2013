import numpy as np
import cv2, cv, math

def hsv_orange_red_threshold(input_image):
    blur_image = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC3)
    cv.Smooth(input_image,blur_image,cv.CV_BLUR, 10, 10)
    proc_image = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC3)
    cv.CvtColor(blur_image, proc_image, cv.CV_BGR2HSV)
    split_image = [cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1),cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1),cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)]
    cv.Split(proc_image, split_image[0],split_image[1],split_image[2], None )

    thresh_0 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    thresh_1 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    thresh_2 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    red_orange = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    cv.Threshold(split_image[1],thresh_0, 128,255,cv.CV_THRESH_BINARY) # > 50% saturation
    cv.Threshold(split_image[0],thresh_1, 220,255,cv.CV_THRESH_BINARY) # > Purple
    cv.Threshold(split_image[0],thresh_2, 10, 255,cv.CV_THRESH_BINARY_INV) # < Yellow-Orange
    cv.Add(thresh_1,thresh_2,red_orange)
    cv.And(red_orange,thresh_0,red_orange)

    return red_orange

PERSPECTIVE_TRANSFORM = np.array(
[[  1.78778569e+00,   9.19775038e-01,  -2.56236007e+02],
 [  2.24321245e-02,   2.98569665e+00,  -3.37687301e+02],
 [  8.04901494e-05,   2.72273026e-03,   1.00000000e+00]])
def homography_transform(img):
    return cv2.warpPerspective(
            img,
            PERSPECTIVE_TRANSFORM,
            (img.shape[1], img.shape[0])
            )

"""
TODO: THESE TWO CONSTANTS SHOULD BE IN THE PARAMETER SERVER; THEY WILL BE
USED IN WHATEVER NODE PROCESSES THE LOGPOLAR TOPIC
"""
logscale = 20
anglescale = 80

def log_polar_transform(img):
    center = (img.width/2, img.height)
    maxradius = math.sqrt((img.width/2)**2 + (img.height)**2)
    width = int(logscale*math.log(maxradius))

    logpolar = cv.CreateImage(
            (width, anglescale),
            cv.IPL_DEPTH_8U,
            3 # why is this need to be 3 here, but 1 in polar_processor.py?
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

