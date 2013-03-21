#!/usr/bin/env python
import cv2, sys, math, cv, pygame
import numpy as np

BLUR_SIZE = (10, 10)

PERSPECTIVE_TRANSFORM = np.array(
[[ -3.38211005e+00,  -3.63700168e+00,   7.24379943e+02],
 [ -4.58408261e-01,  -7.79335168e+00,   9.49241798e+02],
 [ -2.06572848e-03,  -2.00950642e-02,   1.00000000e+00]])

THRESHOLD_BOUNDS = (0.6953125, 0.99609375000000022, 0.13281250000000006, 0.46874999999999994, 0.203125, 0.60156250000000011)

logscale = 20
num_planar_rays = 10.0
anglescale = int(8*num_planar_rays)


PIXEL_HEIGHT_TO_METER_RATIO = 147

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

def makeLogPolar(img):
    center = (img.width/2, img.height)
    maxradius = math.sqrt((img.width/2)**2 + (img.height)**2)
    width = int(logscale*math.log(maxradius))

    logpolar = cv.CreateImage((width, anglescale), cv.IPL_DEPTH_8U, 1)
    cv.LogPolar(img, logpolar, center, logscale, cv.CV_INTER_LINEAR + cv.CV_WARP_FILL_OUTLIERS)
    
    aligned = cv.GetSubRect(logpolar, (0, anglescale/2, width, anglescale/2))
    
    return aligned

def iplToNumpy(source):
    mat = cv.GetMat(source)
    return np.asarray(mat)

def numpyToIpl(source):
    bitmap = cv.CreateImageHeader((source.shape[1], source.shape[0]), cv.IPL_DEPTH_8U, 1)
    cv.SetData(bitmap, source.tostring(), 
               source.dtype.itemsize * 1 * source.shape[1])
    return bitmap

def initWindows(imgs):
    boty = imgs[0].shape[0]+60
    leftx = imgs[0].shape[1]+60
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
    angleWidth = math.pi/num_planar_rays
    rowHeight = anglescale/2/num_planar_rays
    
    maxdist = logpolar.shape[1]
    
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
        
        distance = math.exp(x/float(logscale))/PIXEL_HEIGHT_TO_METER_RATIO
        
        rays.append(Ray(angle, distance, angleWidth))
        
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
        self.blurred = cv2.blur(img, BLUR_SIZE)
        self.thresholded = thresholdImage(self.blurred, THRESHOLD_BOUNDS)
        self.transformed = transformImage(self.thresholded, PERSPECTIVE_TRANSFORM)
        self.logpolar = iplToNumpy(makeLogPolar(numpyToIpl(self.transformed)))

    def calcPlanarData(self):
        self.planarData = makePlanarData(self.logpolar)
        return self.planarData

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
        images = [self.img, self.blurred, self.thresholded, self.transformed, self.logpolar]
        if not self.windowsInit:
            initWindows(images)
            self.windowsInit = True
        dispImages(images)



