#!/usr/bin/env python
import cv2, sys, math, cv, pygame
import numpy as np
from sensor_msgs.msg import LaserScan

BLUR_SIZE = (10, 10)

PERSPECTIVE_TRANSFORM = np.array(
[[  1.97833490e+00,   1.14785392e+00,  -1.55191394e+02],
 [ -6.41195260e-02,   3.66219334e+00,  -2.21460072e+02],
 [ -2.22609309e-04,   7.49904508e-03,   1.00000000e+00]])

THRESHOLD_BOUNDS = (0.55859374999999989, 0.99609375000000011, 0.13281249999999994, 0.51953125, 0.32812499999999994, 0.57421874999999989)

logscale = 20
num_planar_rays = 10.0
anglescale = int(8*num_planar_rays)
angleWidth = math.pi/num_planar_rays
rowHeight = anglescale/2/num_planar_rays

PIXELS_PER_METER = 136.72 
DISTANCE_FROM_HOKUYO = 0.4318 # meters

def transformImage(img, transform):
    return cv2.warpPerspective(img, transform, (img.shape[1], img.shape[0]))

def thresholdImage(img, bounds):
    split_channels = cv2.split(img)
    blue_img = cv2.split(img)[0]
    green_img = cv2.split(img)[1]
    red_img = cv2.split(img)[2]

    flag, redLowerBinImg = cv2.threshold(red_img, bounds[0]*255, 1, cv2.THRESH_BINARY)
    flag, redUpperBinImg = cv2.threshold(red_img, bounds[1]*255, 1, cv2.THRESH_BINARY_INV)
    flag, greenLowerBinImg = cv2.threshold(green_img, bounds[2]*255, 1, cv2.THRESH_BINARY)
    flag, greenUpperBinImg = cv2.threshold(green_img, bounds[3]*255, 1, cv2.THRESH_BINARY_INV)
    flag, blueLowerBinImg = cv2.threshold(blue_img, bounds[4]*255, 1, cv2.THRESH_BINARY)
    flag, blueUpperBinImg = cv2.threshold(blue_img, bounds[5]*255, 1, cv2.THRESH_BINARY_INV)

    mergedImg = cv2.multiply(blueUpperBinImg,\
                 cv2.multiply(blueLowerBinImg,\
                  cv2.multiply(greenUpperBinImg,\
                   cv2.multiply(greenLowerBinImg,\
                    cv2.multiply(redUpperBinImg, redLowerBinImg)))), scale=255)

    return mergedImg

def hsv_threshold(input_image):
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

def makeLogPolar(img):
    center = (img.width/2, img.height)
    maxradius = math.sqrt((img.width/2)**2 + (img.height)**2)
    width = int(logscale*math.log(maxradius))

    logpolar = cv.CreateImage((width, anglescale), cv.IPL_DEPTH_8U, 1)
    cv.LogPolar(img, logpolar, center, logscale, cv.CV_INTER_LINEAR + cv.CV_WARP_FILL_OUTLIERS)
    
    aligned = cv.GetSubRect(logpolar, (0, anglescale/2, width, anglescale/2))
    
    return aligned

def cv2array(im):
  depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
    }

  arrdtype=im.depth
  a = np.fromstring(
         im.tostring(),
         dtype=depth2dtype[im.depth],
         count=im.width*im.height*im.nChannels)
  a.shape = (im.height,im.width,im.nChannels)
  return a

def array2cv(a):
  dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
    }
  try:
    nChannels = a.shape[2]
  except:
    nChannels = 1
  cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),
          dtype2depth[str(a.dtype)],
          nChannels)
  cv.SetData(cv_im, a.tostring(),
             a.dtype.itemsize*nChannels*a.shape[1])
  return cv_im

def iplToNumpy(source):
    mat = cv.GetMat(source)
    return np.asarray(mat)

def numpyToIpl(source):
    bitmap = cv.CreateImageHeader((source.shape[1], source.shape[0]), cv.IPL_DEPTH_8U, 1)
    cv.SetData(bitmap, source.tostring(), 
               source.dtype.itemsize * 1 * source.shape[1])
    return bitmap

def initWindows(imgs):
    boty = imgs[0].shape[0] + 60
    leftx = imgs[0].shape[1] + 60
    locs = [[0,0],[leftx,30],[0,boty],[leftx,boty],[]]

    for i in range(len(imgs)):
        startx = locs[i%4][0]
        starty = locs[i%4][1]
        cv2.namedWindow('img'+str(i))
        cv2.moveWindow('img'+str(i), startx, starty)

def dispImages(imgs):
    for i in range(len(imgs)):
        img = imgs[i]
        cv2.imshow('img'+str(i), img)

class Ray:
    def __init__(self, angleStart, distance, angleWidth):
        self.angleStart = angleStart
        self.distance = distance
        self.angleWidth = angleWidth
    def __repr__(self):
        return str(self.angleStart)+", "+str(self.distance)+", "+str(self.angleWidth)+"\n"

def makePlanarData(logpolar):
    rays = []
    
    maxdist = logpolar.shape[1]
    distance = None
    for angleIndex in range(int(num_planar_rays)):
        angle = math.pi - angleIndex*angleWidth
        row = angleIndex*rowHeight
        
        for x in range(0, int(maxdist)):
            flag = False
            for y in range(int(row), int(row + rowHeight)):
                if logpolar[y,x] > 0:
                    flag = True
                    break
            if flag:
                break
        
        distance = math.exp(x/float(logscale))

        distance = distance/PIXELS_PER_METER + DISTANCE_FROM_HOKUYO
        
        rays.append(Ray(angle, distance, angleWidth))

    rays.append(Ray(0.0, distance, 0.0))
        
    return rays

class ImageProcessor:
    def __init__(self, SIZEX=400, SIZEY=400, DISTANCE_GUI_SCALE=100):
        self.SIZEX = SIZEX
        self.SIZEY = SIZEY
        self.DISTANCE_GUI_SCALE = DISTANCE_GUI_SCALE

        self.refPos = (SIZEX/2, SIZEY/2)
        self.planarData = None

        pygame.init()
        self.window = pygame.display.set_mode((SIZEX, SIZEY))
        self.background = pygame.Surface(self.window.get_size())
        self.background = self.background.convert()
        self.background.fill((0, 0, 0))

        self.windowsInit = False

    def calcPolarImage(self, img):
        self.img = img;
        #self.blurred = cv2.blur(img, BLUR_SIZE)
        #self.thresholded = thresholdImage(self.blurred, THRESHOLD_BOUNDS)
        self.thresholded = hsv_threshold(self.img)
        self.transformed = cv.fromarray(transformImage(np.asarray(self.thresholded), PERSPECTIVE_TRANSFORM))
        #self.logpolar = iplToNumpy(makeLogPolar(numpyToIpl(self.transformed)))
        self.logpolar = np.asarray(makeLogPolar(self.transformed))

    def calcPlanarData(self):
        self.planarData = makePlanarData(self.logpolar)
        return self.planarData

    def getScan(self):
        scan = LaserScan()
        scan.angle_increment = angleWidth
        scan.ranges = [ray.distance for ray in self.planarData]
        scan.ranges = scan.ranges[::-1]
        return scan

    def draw(self):
        if self.planarData != None:
            self.window.blit(self.background, (0,0))

            for i in range(len(self.planarData)) :
                angle = self.planarData[i].angleStart
                dist = self.planarData[i].distance

                x = self.refPos[0] + self.DISTANCE_GUI_SCALE*dist*math.cos(angle)
                y = self.refPos[1] + self.DISTANCE_GUI_SCALE*dist*math.sin(angle)

                pygame.draw.line(self.window, (255, 255, 255), self.refPos, (x, self.SIZEY - y))

        pygame.display.flip()

    def display(self):
        
        if not self.windowsInit:
            self.windowsInit = True
            cv.NamedWindow("img")
            cv.NamedWindow("thresholded")
            cv.NamedWindow('transformed')
            cv2.namedWindow('logpolar')

        cv.ShowImage("img", self.img)
        cv.ShowImage("thresholded", self.thresholded)
        cv.ShowImage('transformed', self.transformed);
        cv2.imshow('logpolar', self.logpolar);
        """
        images = [self.img, self.thresholded, self.transformed, self.logpolar]
        if not self.windowsInit:
            initWindows(images)
            self.windowsInit = True
        dispImages(images)
        """


