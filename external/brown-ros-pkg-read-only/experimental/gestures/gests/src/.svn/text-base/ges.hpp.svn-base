// 10/20 as of tonightt, we get a segfault that seems to have
// something to do with findLump().  Check out the value of
// oldScreenPt in findPerson().  Bounds checking is probably in order,
// but it would be best first to figure out why it is failing.


// Add constructors and destructors in this file.
// Create stubs in .cpp file, test compile.


//  Class to find and return gestures from a selection of video inputs.
//  We use a depth map, and video from the takeshi package that
//  provides estimates of horizontal, vertical, and oscillatory motion
//  at each pixel.

//  Tom Sgouros 8/31/2012
 
#ifndef GES_H
#define GES_H

#include "logit.hpp"

#define NZONES 7
#define NIMGS 3

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <complex>
#include <functional>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <string.h>
#include <map>
#include <utility>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

// syntactic sugar for dealing with IplImages.
#define pindex(arr,x,y) ((uchar *)(arr->imageData + (y)*arr->widthStep))[(x)]

using namespace std;
namespace ublas=boost::numeric::ublas;

// Use this struct to specify a pixel relative to some comparison
// pixel.  The units are in pixels, and the 'comp' value is 1 if the
// specified pixel must be different than the standard and 0 if it
// should be roughly the same.  The tolerance for the comparison must
// be specified elsewhere. 
struct GOffsetPt {
  int x,y;
  int comp;
  GOffsetPt(int xin, int yin, int compin) : x(xin), y(yin), comp(compin) {}
  GOffsetPt() : x(0), y(0), comp(0) {}
};


////////////// Utility class for testing images ///////

// The test images are loaded into the TestImg class, where they can
// be subjected to various tests, such as occupancy of a polygon, or
// value of a pixel.  This class gets sub-classed into GTestImgMono
// and GTestImgRGB, depending on whether we're talking about a mono or
// RGB image.
class GTestImg {

protected:
  IplImage* img;

public:
  GTestImg() : img(NULL) {}

  // The destructor is custom so we can run the OpenCV image release
  // functions.
  ~GTestImg() {
    cvReleaseImage(&img);
  };

  // These are public to make this class sort of parallel in usage
  // to the IplImage class.

  int width, height;

  IplImage* getImg() { return(img); };
  void setImg(IplImage* inImg);

  // Return the value of the array at the given screen location.  This
  // should really be in the parent class, but not sure how to handle
  // the different return values.
  uchar getPixel(const int x, const int y);

};

struct GPxHVO {
  double horiz, vert, osc;
  GPxHVO(double hin, double vin, double oin): 
    horiz(hin), vert(vin), osc(oin) {}
  GPxHVO() : horiz(0.0), vert(0.0), osc(0.0) {}
};

class GTestImgRGB : public GTestImg {

public:

  GPxHVO testRect(const CvPoint ll, const CvPoint ur);

};

#define sindex(st,y,x,i) st[y * width * 2 + x * 2 + i]

class GMotionAnalyzer {
private:
  static const int MAXMS=60000;
  static const int offsets[4][2];

  int width, height;
  bool moving;
  int threshold;
  int ticks;
  int oscGoal, oscTolerance, oscCount, oscArea;
  timeval then;
  int* state;

  IplImage *lastImg, *traceImg;

public:
  GMotionAnalyzer(const int w, const int h) :
    width(w), height(h),
    moving(false),
    threshold(10),
    ticks(0),
    oscGoal(1000), oscTolerance(200), oscCount(4), oscArea(80),
    state(NULL) {

    gettimeofday(&then, NULL);
    state = new int[width * height * 2];
    lastImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    traceImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  }

  ~GMotionAnalyzer() {
    cvReleaseImage(&lastImg);
    cvReleaseImage(&traceImg);
  }

  IplImage* analyzeMotion(const IplImage* img);

};

class GTestImgMono : public GTestImg {

private:
  GMotionAnalyzer* motionAnalyzer;

public:
  GTestImgMono() : motionAnalyzer(NULL) {}

  // Returns T/F indicating whether the comparison pixel (specified
  // with x,y) satisfies the conditions implied by the combination of
  // the offsets array and the tolerance. 
  bool testPixel(const int x, const int y, 
		 const vector<GOffsetPt>& offsets, 
		 const int tolerance);

  // Returns the percentage of the given polygon occupied by pixels of
  // the value given by the pixel at (x,y). For now we just operate on
  // rectangles, specified with the lower left and upper right
  // corners.
  float testRect(const int targetDepth, const int tolerance,
		 const CvPoint ll, const CvPoint ur,
		 const double displacement);

  // Returns a 3-channel RGB image with the R channel indicating
  // horizontal motion, the G channel serving for vertical motion and
  // the B channel having something to do with oscillation.
  IplImage* analyzeMotion();

  // Moving from (x,y) in the direction implied by (xdir, ydir),
  // returns the coordinates of the first deviation of more than
  // tolerance from the value of (x,y). 
  GOffsetPt findDiscontinuity(const int x, const int y, 
			     const int tolerance, 
			     const int xdir, const int ydir);

};

////////////// Person recognition ///////////////
#define TOLERANCE 8
// Used to track a person's location.  Input is a point cloud, from
// which we make (and keep) a depth image.  Available outputs are the
// person's screen location and physical location (in the form of a
// ROS PoseStamped object).  There is also a scratchImg object which
// is mostly just for debugging purposes.  The object also makes the
// depthImg available so that the gesture recognizer can use it.
class GPerson {
private:
  // The scratch image is for drawing contours and doing calculations.
  // It has accessors and mutators so that we can monitor what's going
  // on during debugging.
  IplImage *scratchImg;
  bool detected;

  // These two parameters control the quality of the scan for
  // discontinuities.  They are very specific to the resolution of the
  // camera in question.  The idea is that if a head is x distance
  // from the camera, the width of that head (in pixels) should be a
  // bit less than calibConst/x.
  double calibConst;
  //  double calibOffset;
  int tolerance;
  int minSize;
  double cameraY;

  // Screen coordinates of our best guess of a person's location.  The
  // physical position is also recorded, but is public, see below.
  CvPoint screenPt;
  uchar screenZ;

  bool findPerson(const sensor_msgs::PointCloud2ConstPtr& cloud);
  CvPoint findHeadAndShoulders(const sensor_msgs::PointCloud2ConstPtr& cloud);
  CvPoint findLump(const CvPoint guess, const uchar depthGuess, 
		   const uchar inTolerance);

  geometry_msgs::PoseStamped 
  getPhysicalLocation(const CvPoint screenPt,
		      const sensor_msgs::PointCloud2ConstPtr& cloud);

public:
  GPerson() : 
    scratchImg(NULL), 
    detected(false), 
    calibConst(-1.5), //-.6
    //    calibOffset(125.0),
    tolerance(TOLERANCE),  // 4
    minSize(50),
    cameraY(1.15), 
    screenZ(0) { screenPt = cvPoint(0, 0); }

  // Should this be private?
  // Estimates of the person's location, in physical units.  Note that
  // the headers of this data types are continually updated as point
  // cloud data comes in, but the data in them only becomes valid when
  // a person has been detected.  In other words, don't publish them
  // without checking the detected flag.
  geometry_msgs::PoseStamped physPt;

  // Input image from which the position is inferred. 
  GTestImgMono depthImg;

  void setCalibConst(const double inConst) { calibConst = (double)inConst; };
  void setTolerance(const int inTolerance) { tolerance = inTolerance; };
  void setMinSize(const int inMinSize) { minSize = inMinSize; };
  void setCameraY(const double inCameraY) { cameraY = inCameraY; };

  double getCalibConst() { return calibConst; };
  //  double getCalibOffset() { return calibOffset; };
  int getTolerance() { return tolerance; };
  int getMinSize() { return minSize; };
  int getCameraY() { return cameraY; };

  int getSeg(int depth) {
    if (depth > 0) {
      return((int)(calibConst/((float)depth)));
    } else {
      return(0);
    }
  };
  int getScreenDepth() { return(screenZ); };

  // Flag to indicate whether or not a person has been detected. 
  bool isDetected() { return(detected); };

  CvPoint getScreenLocation();

  void checkOutCloud(const sensor_msgs::PointCloud2ConstPtr& cloud);

  void makeDepthImg(const sensor_msgs::PointCloud2ConstPtr& cloud);
  // Note that this returns the image, not the GTestImg.  For convenience.
  IplImage* getDepthImg();

  void setScratchImg(IplImage* inScratchImg);
  IplImage* getScratchImg();
  
  // debug
  int measureHead(const CvPoint guess, 
		  const uchar depthGuess,
		  const uchar inTolerance);
};


// Holds the coordinates of a "zone" in which we'll be looking for a
// gesture.  Only handles rectangular zones, and the units are in
// terms of the scaling factor, also called "seg" pixels. That is, a
// value of 1 means seg pixels.
//
// The depth offset says that the zone in question is in a different
// plane than the reference plane.  The reference plane is usually
// about the user's collarbone, and if we're looking for action in
// front of the body, we need a slightly positive depthOffset.
typedef struct gZone {

  CvPoint2D32f ll, ur;
  double depthOffset;

  gZone(double llx, double lly, double urx, double ury, double ddo) :
    ll(cvPoint2D32f(llx, lly)), 
    ur(cvPoint2D32f(urx, ury)), 
    depthOffset(ddo) {}
  gZone(): ll(cvPoint2D32f(0.0, 0.0)), 
	   ur(cvPoint2D32f(0.0, 0.0)), 
	   depthOffset(0.0) {}

} GZone;

// Holds a zone threshold, and the meaning of that threshold (not to
// exceed, must exceed, don't care).
typedef struct gThreshold {
  float threshold;
  // This array indicates how to treat the zone values. 
  //   1 = threshold value is a lower bound
  //   0 = ignore this value
  //  -1 = threshold is an upper bound 
  int treatment;
} GThreshold;


// A convenience struct for packaging a gesture and a measure of our
// confidence that it has been observed.
typedef struct gReturnedGesture {

  string gesture;
  float confidence;

  gReturnedGesture(string inGes, float inConf) 
    : gesture(inGes),confidence(inConf) {}

} GReturnedGesture;


// A class to manage a growing list of occupancy data tuples.

class GPoses {

private:

  // The training data is stored in here.  The training data contains
  // the pose names each tuple represents.  See the logit.cpp code for
  // more, including the file format for the training data.
  GSamples samples;

  // This is the growing data record.  It's capped at maxSize records,
  // so the addition of the (maxSize + 1)th record (via addPose())
  // causes the first to be popped off the front.
  list<ublas::matrix<double> > poseRecord;

  // This member defines a mapping between pose names and the names of
  // the gestures of which they are a part.  For the moment (10/12)
  // the poses are named for the gestures to which they correspond,
  // but ultimately we will likely subdivide the gestures into
  // multiple poses to help with the clustering of training data
  // points. 
  map<string, string> gestureNames;


public:
  GPoses() {};
  GPoses(string fileName) {
    samples = GSamples(fileName);

    samples.train();

    gestureNames.insert(pair<string,string>("forward", "forward"));
    gestureNames.insert(pair<string,string>("stop", "stop"));
    gestureNames.insert(pair<string,string>("left", "left"));
    gestureNames.insert(pair<string,string>("right", "right"));
    gestureNames.insert(pair<string,string>("look", "look"));
    gestureNames.insert(pair<string,string>("release", "release"));
    gestureNames.insert(pair<string,string>("null", "null"));
    gestureNames.insert(pair<string,string>("nothing", "nothing"));
    gestureNames.insert(pair<string,string>("cringe", "cringe"));
  };

  static const unsigned int maxSize = 8;

  void addPose(ublas::matrix<double>& occupancy);

  // Returns the current best guess about the gesture being performed.
  GReturnedGesture consensus();



};


// This class is used to manage the data used to identify a gesture.
// The original sources of information are these:
//
//   - A depth image.  This is absorbed with the checkOutCloud()
//     method, and is (inside there) shared with the GPerson object,
//     for person location.
//
//   - A collection of training data used to define poses that belong
//     to the gestures we're looking to identify.  The training data
//     consist of recorded observations of occupancy, horizontal
//     motion, and vertical motion within several "zones" defined
//     around a person, along with string labels for the gesture that
//     was being performed at the time.
//
// These are used to develop more numbers.
//
//   - The depth image is used to produce a time series of values for
//     the zones.  Each measurement corresponds to a "pose" component
//     of a gesture.  Note that because the pose data includes motion,
//     you shouldn't really think of it as a stationary pose.  It's
//     more of a snapshot, which might be blurry in a specific way.
//
//   - The training data is used in a logistic regression to produce a
//     classifier for the poses.  The output of the classifier is used
//     to evaluate the likelihood that a specific gesture is being
//     performed at the moment.
//
// To use the class, give it a file name containing training data, and
// then put checkOutCloud() in some ROS client to listen to depth
// images. 
//
class Gesture {
private:
  // The video dimensions.  All the images must be this size. 
  int vWidth, vHeight;

  // These images are the reference data for all the location and
  // gesture calculations.  They are not changed by this class.  There
  // are two images, one is a depth map from a stereo camera or
  // kinect, and is stored in the GPerson object.  The other is a
  // composite image with motion values, modeled after the takeshi
  // package.
  //  GTestImgMono depthImg; 
  GTestImgRGB motionImg;

  // Scratch pad for location calculations. 
  //  IplImage* scratchImg;

  // Monitor this image for information about how the calculations are
  // going.  It has rectangles superimposed on the depth image, to help
  // tune. 
  IplImage* dbgImg;

  // The name of the gesture we're currently thinking is in the view.
  // If there is none, this will have the value of "none".
  std::string currentGesture;
  float currentConfidence;

  // This is essentially the same as the tolerance in the GPerson
  // class object.  Not quite sure how best to initialize it, but this
  // will do for now.
  int tolerance;

  // This is the actual data derived from the depth image.  A list of
  // NZONES x NIMGS-tuples.
  //  vector<vector<double> > occupancy;
  ublas::matrix<double> occupancy;

  // Gestures are defined as a combination of occupancy in "zones",
  // generally around a detected person for each of the relevant video
  // sources.  For example, the "stop" gesture involves a stationary
  // fist in the air, which means one of the zones on either side of
  // the head is more occupied than usual in the depth image, and the
  // movement images are fairly quiet.
  //
  // There are seven zones and we expect three sources of image
  // information (one measuring depth, and the other two measuring
  // motion: horizontal, vertical.  There is also an oscillatory
  // analysis that is not implemented as of 10/12.)
  //
  // This is initialized in ges.cpp.  Not quite sure why the rules
  // seem to require that, but they appear to.
  static const GZone zoneLocs[NZONES];

  // A copy of the CvBridge object to use for conversions.
  //sensor_msgs::CvBridge bridge;

  // Using the data currently on hand, update the current estimate of
  // whether there is a gesture to be detected out there and what it
  // might be, if anything.
  void updateEstimates();

  // Use the currently available data to guess the actual physical
  // location of the person making the gesture (if any).
  void locatePerson();

  GPoses gPoses;


public:
  // This class needs the image dimensions to initialize correctly.
  // It makes little sense to use it without some defined gestures,
  // but those can be added subsequently via the add and del methods.
  //Gesture(<const int inWidth, const int inHeight);
  Gesture(string trainingFileName) : vWidth(0), vHeight(0), 
			     currentGesture("none"), 
			     currentConfidence(0.0),
			     tolerance(TOLERANCE) {
    occupancy = ublas::matrix<double> (NZONES, NIMGS);
    gPoses = GPoses(trainingFileName);
  }

  // The destructor is custom so we can run the OpenCV image release
  // functions.
  //~Gesture();

  // This contains our best estimate of the location of the person
  // giving the gestures, (the user) and some aspects of his or her
  // size.  This should probably be private, is public for now in
  // order to be able to return the person's pose.
  GPerson person;

  // Use this function to get the current guess of what kind of
  // gesture is being made, if any. 
  GReturnedGesture getGesture();

  // Modify the list of gestures we're searching for.
  bool addGesture(string inGesture);
  bool delGesture(string gestureName);

  // Creates a zone and parks it in the zones vector.
  //bool setZone(int iZone, double llx, double lly, double urx, double ury);
  //vector<GZone> getZones();

  // Return the debug image, presumably preparatory to publishing it. 
  IplImage* getDbgImg();

  // Mutators for the image data.  First one takes data directly from
  // a ROS point cloud messag.
  void checkOutCloud(const sensor_msgs::PointCloud2ConstPtr& cloud);

  // Returns a formatted string containing the occupancy data.
  string occupancyString();
};




#endif
