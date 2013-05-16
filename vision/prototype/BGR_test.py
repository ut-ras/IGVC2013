#!/usr/bin/env python

import cv

ll = "/home/granny/ros/ros-pkg/IGVC2013/vision/prototype/Johnny5_BGR_Test.png"
img = cv.LoadImageM(ll)

#cv.ShowImage("Original", img )

bgr_split = [cv.CreateMat(img.rows, img.cols, cv.CV_8UC1) for x in range (3)]

cv.Split(img, bgr_split[0], bgr_split[1], bgr_split[2], None)

cv.ShowImage("Blue",  bgr_split[0])
cv.ShowImage("Red",   bgr_split[1])
cv.ShowImage("Green", bgr_split[2])

two_blue = cv.CreateMat(img.rows, img.cols,cv.CV_8UC1)
gray = cv.CreateMat(img.rows, img.cols,cv.CV_8UC1)

cv.Add(bgr_split[0], bgr_split[0], two_blue)
cv.Sub(two_blue, bgr_split[1], gray)

cv.ShowImage("Gray", gray)

cv.WaitKey(100000)
cv.DestroyWindow("Blue")
cv.DestroyWindow("Red")
cv.DestroyWindow("Green")
cv.DestroyWindow("Gray")
