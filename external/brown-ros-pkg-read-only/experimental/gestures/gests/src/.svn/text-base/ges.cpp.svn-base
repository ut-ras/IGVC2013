#include "ges.hpp"

// TBD: What exactly is the range of occupied levels returned from the
// analyze motion routine?  Seems like they might be routinely far
// less than one.
//

// Person finding more stable, false positives a problem (probably in
// findLump -- follow the logic if there is no lump at the guess.y
// value.

// Logic in findPerson() is not correct.  See ******

void GPoses::addPose(ublas::matrix<double>& occupancy) {
  poseRecord.push_back(occupancy);
  if (poseRecord.size() > maxSize) {
    poseRecord.pop_front();
  }
}

// Returns our best guess of what gesture is currently being performed.
GReturnedGesture GPoses::consensus() {

  if (!samples.isTrained()) { 
    return GReturnedGesture("null", 0.0);
  }

  ublas::matrix<double> testData = 
    ublas::matrix<double> (maxSize, NZONES * NIMGS);

  // The poses are stored as a series of N 7x3 matrices, but sent to
  // the classifier as a Nx21 matrix.  Convert.
  int iRow = 0;
  for (list<ublas::matrix<double> >::iterator it = poseRecord.begin();
       it != poseRecord.end(); it++) {

    ublas::matrix_row<ublas::matrix<double> > testRow (testData, iRow++);

    for (int iZone = 0; iZone < NZONES; iZone++) {
      for (int jImg = 0; jImg < NIMGS; jImg++) {
	testRow[iZone * NIMGS + jImg] = (*it)(iZone, jImg);
      }
    }
  }

  // Make the predictions.
  vector<string> poses = samples.predict(testData);

  // Translate the pose names into gesture names.  For now, this is an
  // identity, but we hope it won't stay that way.
  vector<string> gestures;
  for (vector<string>::iterator it = poses.begin(); it < poses.end(); it++) {
    if ((*it) != "NA") 
      gestures.push_back(gestureNames.find(*it)->first);
  }

  // Now vote.  Majority rules.
  map<string, int> counter;

  for (vector<string>::iterator it = gestures.begin(); 
       it < gestures.end(); it++) {
    counter[ *it ]++;
  }

  int max = 0;
  string outString;
  for (map<string, int>::iterator it = counter.begin();
       it != counter.end(); it++) {
    if (it->second > max) {
      max = it->second;
      outString = it->first;
    }
  }

   
  if (max > maxSize/2) {
    return GReturnedGesture(outString, ((double)max)/((double)maxSize));
  } else {
    return GReturnedGesture("null", 0.0);
  }
}


GReturnedGesture Gesture::getGesture() {

  return gPoses.consensus();

};

// Modify the list of gestures we're searching for.
bool Gesture::addGesture(string inGesture) {
  return(1);
};

bool Gesture::delGesture(string gestureName) {
  return(1);
};

// Return the debug image, presumably preparatory to publishing it. 
IplImage* Gesture::getDbgImg() {
  return(dbgImg);
};

// Mutators for the image data. 
void Gesture::checkOutCloud(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  // Get the image dimensions
  vWidth = cloud->width;
  vHeight = cloud->height;

  person.checkOutCloud(cloud);

  // Create the image structures...
  // If there isn't a debug image of the right size, make one.
  if ((!dbgImg) || 
      (person.depthImg.width != dbgImg->width) || 
      (person.depthImg.height != dbgImg->height)) {
    dbgImg = cvCreateImage(cvSize(person.depthImg.width, 
				  person.depthImg.height), 
			       IPL_DEPTH_8U, 3);
  }

  cvCvtColor(person.getDepthImg(), dbgImg, CV_GRAY2RGB);

  // If we detected a person, draw a rectangle on him or her, and then
  // update our estimates of the gesture displayed, if there is one.
  if (person.isDetected()) {

    CvPoint personPt = person.getScreenLocation();

    cvRectangle(dbgImg, 
		cvPoint(personPt.x - 10, personPt.y - 10),
		cvPoint(personPt.x + 10, personPt.y + 10), 
		cvScalar(255,0,0), -4);

    updateEstimates();

    // It might be worth revisiting the person-identification now.
    // Could finding a valid gesture with high confidence be an
    // additional piece of data to increase the confidence of the
    // person identification?  Or would that only be worth it with a
    // skeleton-based person-identification method?

  }
}


//vector<GZone> const zoneLocs (NZONES)
const GZone Gesture::zoneLocs[NZONES] = {

  // user right hand, low and in
  gZone(-1.5,  0.0, -0.3, 1.5, -1.0),
  // user left hand, low and in
  gZone( 0.3,  0.0,  1.5, 1.5, -1.0),
  // user right hand, extended 
  gZone(-3.0, -0.5, -1.0, 0.5, 0.0),
  // user left hand, extended out
  gZone( 1.0, -0.5,  3.0, 0.5, 0.0),
  // user right hand, diagonally up
  gZone(-2.5, -2.0, -0.5, -0.5, 0.0),
  // user left hand, diagonally up
  gZone( 0.5, -2.0,  2.5, -0.5, 0.0),
  // over head
  gZone(-0.5, -2.5,  0.5, -1.0, 0.0)
}; 


void Gesture::updateEstimates() {

  // Only use this if a person has been detected.

  CvPoint loc = person.getScreenLocation();
  int depth = person.getScreenDepth();
  int seg = person.getSeg(depth);

  motionImg.setImg(person.depthImg.analyzeMotion());

  // Survey the occupancy measure of each zone.

  for (int i = 0; i < NZONES; i++) {

    CvPoint ll= cvPoint(loc.x + zoneLocs[i].ll.x * seg, 
			loc.y + zoneLocs[i].ll.y * seg);
    CvPoint ur= cvPoint(loc.x + zoneLocs[i].ur.x * seg, 
			loc.y + zoneLocs[i].ur.y * seg);

    // The occupancy array contains the actual estimates referred to
    // in the function name.

    //int dc = (int)(zoneLocs[i].depthOffset * seg);
    occupancy(i, 0) = 
      person.depthImg.testRect(depth, tolerance, ll, ur,
			       zoneLocs[i].depthOffset);

    GPxHVO motion = motionImg.testRect(ll, ur);
    occupancy(i, 1) = motion.horiz;
    occupancy(i, 2) = motion.vert;
    //occupancy(i, 3) = motion.osc;
      
    // Draw our rectangles
    int border; 
    if (occupancy(i, 0) > 0.8) {
      border = -2;
    } else {
      border = 5;
    }
    cvRectangle(dbgImg, 
		cvPoint(loc.x + zoneLocs[i].ll.x * seg, 
			loc.y + zoneLocs[i].ll.y * seg),
		cvPoint(loc.x + zoneLocs[i].ur.x * seg, 
			loc.y + zoneLocs[i].ur.y * seg),
		cvScalar(32+(int)(occupancy(i, 0) * 127),
			 32, //128 + (int)(occupancy(i, 1) * 127),
			 32), // + (int)(occupancy(i, 0) * 127)), 
		border);

    // draw bouncing rectangle
    int xmin = loc.x + zoneLocs[i].ll.x * seg;
    int xmax = loc.x + zoneLocs[i].ur.x * seg;
    int xmid = (xmin + xmax)/2;
    int dimx = (xmax - xmin)/8;
    int ymin = loc.y + zoneLocs[i].ll.y * seg;
    int ymax = loc.y + zoneLocs[i].ur.y * seg;
    int ymid = (ymin + ymax)/2;
    int dimy = (ymax - ymin)/8;
      
    CvPoint llp = cvPoint((xmid - dimx) + 10*
			  occupancy(i, 1) * (xmid - xmin - dimx),
			  (ymid - dimy) + 10*
			  occupancy(i, 2) * (ymid - ymin - dimy));
    CvPoint urp = cvPoint((xmid + dimx) + 10*
			  occupancy(i, 1) * (xmid - xmin - dimx),
			  (ymid + dimy) + 10*
			  occupancy(i, 2) * (ymid - ymin - dimy));
    
    cvRectangle(dbgImg, llp, urp, 
		cvScalar(128+(int)(occupancy(i, 0) * 127),
			 0, //128 + (int)(occupancy(i, 1) * 127),
			 128), // + (int)(occupancy(i, 0) * 127)), 
		6);
  }

  // Add the current data to our record of the poses.
  gPoses.addPose(occupancy);

  // Return the best guess.  The confidence level is packaged with the
  // string.
  GReturnedGesture conGen = gPoses.consensus();

  // Label the image here, if desired.
  //   int headWidth = person.measureHead(loc, depth, 8);

  //   CvPoint leftDot = cvPoint(loc.x - (headWidth/2), loc.y - (seg/2));
  //   CvPoint rightDot = cvPoint(loc.x + (headWidth/2), loc.y - (seg/2));
  //   cvLine(dbgImg, leftDot, rightDot, cvScalar(255,0,0), 4);

  // Use the guessed gesture for a label if the confidence measure is
  // high enough.
  int lineWidth = 1;
  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 1.5, 1.5, 0, lineWidth);
  //   char txt[25];
  //   sprintf(txt, "W:%d", headWidth);
  if (conGen.confidence > 0.5)
    cvPutText(dbgImg, conGen.gesture.c_str(), cvPoint(20, 450), 
	      &font, cvScalar(200,0,0));
  
  return;
};

string Gesture::occupancyString() {

  string buffer = string();
  char zoneString[30];

  for (int i = 0; i < NZONES; i++) {
    sprintf(zoneString, "%9.6f,%9.6f,%9.6f,", 
	    occupancy(i, 0), occupancy(i, 1), occupancy(i, 2));

    buffer.append(zoneString);
  }

  return(buffer);

}

CvPoint GPerson::getScreenLocation() {
  return(screenPt);
};

IplImage* GPerson::getDepthImg() {
  return(depthImg.getImg());
}

void GPerson::makeDepthImg(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  IplImage* raw = cvCreateImage(cvSize(cloud->width, cloud->height), 
				IPL_DEPTH_8U, 1);

  const unsigned char* start = &(cloud->data[0]);
  const unsigned char* cursor = start;

  int j = 0;
  for (uint i = 0; i < cloud->data.size(); i += cloud->point_step, j++) {
    cursor = start + i + 8;
    // offset 0 seems to be a physical x and 4 a physical y.  Are
    // these worth saving for later? *(see part below where we get
    // physical location).
    float z = *(reinterpret_cast<const float*>(cursor)); //offset 8
    int zi = (z / 15.0) * 255.0; // Is this setting a max distance?
    if (isnan(z)) zi = 0;
    if (zi > 255) zi = 255;
    if (zi < 0) zi = 0;
    ((uchar*)raw->imageData)[j] = zi;
  }

  // Now we have a grayscale image that corresponds to the depth data.
  // Feed it to the GTestImg class for convenience of testing.
  depthImg.setImg(raw);
}

void GPerson::checkOutCloud(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  // Make a GTestImg from the input cloud.
  makeDepthImg(cloud);

  // Got all the images and arrays ready?  See if there's someone
  // there.
  detected = findPerson(cloud);

  if (detected) {
    // If we saw a person, see if we can figure out his or her
    // physical location.  This involves finding the nearest point in
    // the cloud to the screen coordinates of the person, and just
    // reading the physical location value out of the point cloud
    // data.
    physPt = getPhysicalLocation(getScreenLocation(), cloud);
  } else {
    // No person detected.
    physPt.pose.position.x = 0.0;
    physPt.pose.position.y = 0.0;
    physPt.pose.position.z = 0.0;
  }
};

// Returns a position based on the current cloud data.
// Inputs: inPt, cloud
geometry_msgs::PoseStamped 
GPerson::getPhysicalLocation(const CvPoint inPt,
			     const sensor_msgs::PointCloud2ConstPtr& cloud) {

  geometry_msgs::PoseStamped p;

  p.header = cloud->header;

  const unsigned char* start = &(cloud->data[0]);
    
  // Width of the neighborhood to look in.  This shouldn't be too big,
  // else we wind up far away from the target point in the x direction
  // while being quite close in the y direction.
  const int nbd = 10; 

  // We survey the neighborhood for a valid point.  Ideally this loop
  // should start at the target point (screen coordinates) and circle
  // outward until we find a point with valid location data.  What we
  // have here is a very rough approximation of such a spiral.  Not
  // all points have valid data; we're just looking for the closest
  // point to our target that does, and this rough spiral more or less
  // does that.
  int y = inPt.y;
  for (int j = 0; j < nbd; j++, y = inPt.y + (j * pow(-1,j))) {
    if (y < 0 || y >= ((int)cloud->height)) continue;
    int x = inPt.x;
    for (int i = 0; i < nbd; i++, x = inPt.x + (i * pow(-1,i))) {
      if (x < 0 || x >= ((int)cloud->width)) continue;
      const uchar* cursor = start + 
	(y * cloud->width * cloud->point_step) + 
	(x * cloud->point_step);

      // Check if there's data at this cursor location.
      float space = *(reinterpret_cast<const float*>(cursor));
      if (isnan(space)) continue;
      
      // If we're here, we passed the valid data screen, so record
      // the position and get out.

      // X value is in position 0
      p.pose.position.x = *(reinterpret_cast<const float*>(cursor));
      // Y value is in position 4
      p.pose.position.y = *(reinterpret_cast<const float*>(cursor + 4));
      // Z in position 8
      p.pose.position.z = *(reinterpret_cast<const float*>(cursor + 8));

      // found a valid point, so get out
      return(p);
    }
  }
  // Should not be here.
  ROS_INFO("ERROR in location finding: nbd too small.");
  p.pose.position.x = 0.0; p.pose.position.y = 0.0; p.pose.position.z = 0.0;
  return(p);
};

IplImage* GPerson::getScratchImg() {
  return(scratchImg);
}

void GPerson::setScratchImg(IplImage* inScratchImg) {
  scratchImg = inScratchImg;
}

// Use whatever new data there is to locate a person in the image.
bool GPerson::findPerson(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  // Inputs: depthImg, cloud, screenPt, screenZ
  // Outputs: updated screenPt, screenZ

  // 9/13/12: This still isn't very stable.  Basically it's only
  // looking for the biggest area of points that satisfy the
  // constraints specified in the offsets array.  Boxes on the floor
  // and chairs work quite well, along with the head-and-torso
  // patterns it's designed for.  The offsets points can certainly be
  // tuned, but there are probably secondary pieces of evidence that
  // need to be added: previous position, is something gesturing,
  // height above floor, existence of roundish thing (head), etc etc.

  // If there isn't a scratch image of the right size, make one.  The
  // scratch image is global to this class for debugging purposes.
  if ((!scratchImg) || 
      (depthImg.width != scratchImg->width) || 
      (depthImg.height != scratchImg->height)) {
    scratchImg = cvCreateImage(cvSize(depthImg.width, 
				      depthImg.height), 
			       IPL_DEPTH_8U, 1);
  }

  cvSetZero(scratchImg);

  // What follows here is a sort of simplistic Bayesian reasoning.  We
  // look for the head-and-shoulders, but we don't expect it to move
  // far or fast from one frame to another.  (And mostly we expect it
  // to move horizontally, if it moves at all.)  We also look to see
  // if there's a mass under or near where the last known measurement
  // was.
  //
  // tbd: More formal Bayesian analysis.
  //      Other data: motion anomaly? color field?  
  //                  find circular objects (head)?
  CvPoint oldScreenPt = screenPt;
  uchar oldScreenZ = screenZ;

  CvPoint headLoc = findHeadAndShoulders(cloud);

  cvRectangle(scratchImg, 
	      cvPoint(headLoc.x - 10, headLoc.y - 10),
	      cvPoint(headLoc.x + 10, headLoc.y + 10), 
	      cvScalar(100,100,100), -4);

  CvPoint lumpLoc = findLump(oldScreenPt, oldScreenZ, tolerance);

  cvRectangle(scratchImg, 
	      cvPoint(lumpLoc.x - 10, lumpLoc.y - 10),
	      cvPoint(lumpLoc.x + 10, lumpLoc.y + 10), 
	      cvScalar(200,200,200), -4);

  CvFont font;
  double hScale = 1.5;
  double vScale = 1.5;
  int lineWidth = 1;
  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, hScale, vScale, 0, lineWidth);
  char txt[50];

  // This logic is not correct *******
  if (headLoc.x || headLoc.y) {
    if (lumpLoc.x || lumpLoc.y) {
      screenPt = cvPoint((headLoc.x + lumpLoc.x)/2, (headLoc.y + lumpLoc.y)/2);
    } else {
      screenPt = headLoc;
    }
    screenZ = depthImg.getPixel(screenPt.x, screenPt.y);

    sprintf(txt, "Za:%d", screenZ);
    cvPutText(scratchImg, txt, cvPoint(20, 450), &font, cvScalar(200,200,200));

    return(true);
  } else if (lumpLoc.x || lumpLoc.y) {
    screenPt = lumpLoc;
    screenZ = depthImg.getPixel(screenPt.x, screenPt.y);

    sprintf(txt, "Zb:%d", screenZ);
    cvPutText(scratchImg, txt, cvPoint(20, 450), &font, cvScalar(100,100,100));

    return(true);
  } else {
    screenPt = cvPoint(0,0);
    screenZ = 0;
    return(false);
  }
}

// This is really just for debugging.  As of 10/03, is not necessary.
int GPerson::measureHead(const CvPoint guess, 
			const uchar depthGuess,
			const uchar inTolerance) {
  int halfSeg = getSeg(depthGuess)/2;
  if (guess.y < 0) {
    return (0);
  }

  CvPoint pt = findLump(cvPoint(guess.x, guess.y - halfSeg), 
			depthGuess, inTolerance);

  int right = guess.x;
  while ((right < depthImg.width) &&
	 abs(depthImg.getPixel(right, guess.y - halfSeg) - depthGuess) 
	 < inTolerance) right++;

  int left = guess.x;
  while ((left > 0) && 
	 (abs(depthImg.getPixel(left, guess.y - halfSeg) - depthGuess) 
	  < inTolerance)) left--;

  return(right - left);
}

// Go to the guess point, and scan across for a person-size mass at
// approximately the same location.  Make a rough guess of its center
// and return that point.  Hidden input: depthImg.
CvPoint GPerson::findLump(const CvPoint guess, 
			  const uchar depthGuess, 
			  const uchar inTolerance) {

  vector<int> lefts;
  vector<int> rights;

  // Find first non-zero entry.
  int i = 0;
  while (!depthImg.getPixel(i, guess.y)) i++;

  bool measuring = false;  // Becomes true when we find a lump.
  for (i++ ; i < depthImg.width; i++) {
    if (!depthImg.getPixel(i, guess.y)) continue; // Skip the zeros.

    if (abs(depthImg.getPixel(i, guess.y) - depthGuess) < inTolerance) {
      if (!measuring) {
	measuring = true;
	lefts.push_back(i);
      }
    } else {
      if (measuring) {
	measuring = false;
	rights.push_back(i);
	if (lefts.empty()) lefts.push_back(0);
      }
    }
  }

  if (rights.size() <  lefts.size()) rights.push_back(depthImg.width);
  if (rights.size() != lefts.size()) ROS_INFO("TROUBLE with lump finder.");


  // Should have two equal size vectors here, one recording the left
  // side of a lump and one the right.
  int mid = 0;
  int dist = 100000;
  for (uint j = 0; j < lefts.size(); j++) {
    int testMid = (lefts[j] + rights[j]) / 2;
    if (abs(guess.x - testMid) < dist) {
      mid = testMid;
      dist = abs(guess.x - testMid);
    }
  }

  if (mid <= 0 || mid >= depthImg.width) {
    // ROS_INFO("out of bounds error (mid:%d, wid:%d)", mid, depthImg.width);
    return(cvPoint(0, 0));
  } else {
   return(cvPoint(mid, guess.y));
  }
}

// Input: depthImg, cloud
// explicit output
CvPoint 
GPerson::findHeadAndShoulders(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  // The way we look for a person is to look for points that satisfy
  // the constraint set up by the offsets array.  The largest
  // collection of such points we hypothesize to be approximately at a
  // person's collarbone.
  vector<GOffsetPt> offsets (7);
  int seg = -1;
  int oldSeg = seg;

  for (int x = 0; x < depthImg.width; x++) {
    for (int y = 0; y < depthImg.height; y++) {
      uchar depth = depthImg.getPixel(x, y);
      int seg = getSeg(depth);

      if (seg > 0) {
	if (seg != oldSeg) {
	  offsets[0] = GOffsetPt(-seg,-seg/2, 1);
	  offsets[1] = GOffsetPt(   0,-seg/2, 0);
	  offsets[2] = GOffsetPt( seg,-seg/2, 1);
	  offsets[3] = GOffsetPt(-seg, seg/2, 0);
	  offsets[4] = GOffsetPt(   0, seg/2, 0);
	  offsets[5] = GOffsetPt( seg, seg/2, 0);
	  offsets[6] = GOffsetPt(   0,   seg, 0);
	  oldSeg = seg;
	}

	// tbd: Making the return from testPixel multi-valued might be
	// a way to distinguish one person from another.  That is, if
	// we return the depth, and record that in the scratch image,
	// the contour around the closest *and* biggest contour can be
	// used.
	if (depthImg.testPixel(x, y, offsets, tolerance)) {
	  pindex(scratchImg, x, y) = 255;
	}
      }
    }
  }

  // We've identified all the spots that might indicate a head atop a
  // torso.  Now we measure them all by drawing a contour around them.
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* comps;

  cvFindContours(scratchImg, storage, &comps, 
		 sizeof(CvContour), CV_RETR_CCOMP, 
		 CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  if (comps) {
    IplImage* cntr = cvCreateImage(cvSize(scratchImg->width,
					  scratchImg->height), 
				   IPL_DEPTH_8U, 3);
    cvSetZero(cntr);
    cvDrawContours(cntr, comps, CV_RGB(255,255,255),
		   CV_RGB(255,0,0),8,1);

    // Which one is the person?  Could be the biggest, or it could be
    // the one nearest the last known person location.
    int size = 0;
    CvContour* biggest = NULL;

    double distance = 1.0e35;
    CvContour* closest = NULL;

    for (CvSeq* comp = comps; comp != NULL; comp = comp->h_next) {
      CvContour* contour = (CvContour*) comp;

      // Is this contour tall enough to be a realistic guess?
      CvPoint testPt = cvPoint(contour->rect.x + (contour->rect.width/2),
			       contour->rect.y + (contour->rect.height/2));

      geometry_msgs::PoseStamped p = getPhysicalLocation(testPt, cloud);
      // Cloud points are in the frame /openni_rgb_optical_frame.  We
      // cheat a little bit here and do a seat-of-the-pants conversion
      // to the robot frame in order to save some time.
      double testHt = cameraY - p.pose.position.y;
      if (testHt < 0.8 || testHt > 2.5) continue;

      // Is this contour bigger than the last?
      int sizeP = contour->rect.width * contour->rect.height;
      if (sizeP < minSize) continue;
      if (sizeP < size) continue;
      size = sizeP;
      biggest = contour;

      // We cheat a little bit and look to see if this one is closer
      // than the last to the last known position of a person?
      if (detected) {

	int xmid = contour->rect.x + contour->rect.width/2;
	int ymid = contour->rect.y + contour->rect.height/2;

	double d = sqrt(pow((double)(screenPt.x - xmid), 2.0) +
			pow((double)(screenPt.y - ymid), 2.0));

	if (d < distance) {
	  distance = d;
	  closest = contour;
	}
      }
    }

    // Put the drawn contour into scratchImg for monitoring purposes.
    cvCvtColor(cntr, scratchImg, CV_RGB2GRAY);

    // Think we've found a person.  Record it and get out.
    if (biggest) {

      // If biggest isn't a lot bigger than closest, go with closest.
      if (closest) {
	int closestSize = closest->rect.height;
	int biggestSize = biggest->rect.height;

	if (biggestSize > closestSize * 2) {
	  return cvPoint(biggest->rect.x + (biggest->rect.width/2),
			 biggest->rect.y + (biggest->rect.height/2));
	} else {
	  return cvPoint(closest->rect.x + (closest->rect.width/2),
			 closest->rect.y + (closest->rect.height/2));
	}
      } else {
	return cvPoint(biggest->rect.x + (biggest->rect.width/2),
		       biggest->rect.y + (biggest->rect.height/2));
      }

    } else {

      return cvPoint(0, 0);
    }

  } else {

    return cvPoint(0, 0);
  }
};

// Returns the value of the pixel at (x,y).  Note that for the mono
// class this is expected to be an unsigned char, while for the RGB
// class it will be three of them (as CvScalar, I believe).
uchar GTestImg::getPixel(const int x, const int y) {
  return pindex(img, x, y);
}

// Loads a new image.  Checks to see if the new image is the same size
// as the old.
void GTestImg::setImg(IplImage* inImg) {
  if ((!img) || 
      (inImg->width != img->width) || 
      (inImg->height != img->height)) {
    img = cvCreateImage(cvSize(inImg->width, inImg->height), 
			inImg->depth, inImg->nChannels);
  }

  width = inImg->width;
  height = inImg->height;

  cvCopy(inImg, img);
};

bool GTestImgMono::testPixel(const int x, const int y, 
			     const vector<GOffsetPt>& offsets, 
			     const int tolerance) {
  vector<GOffsetPt>::const_iterator iter;

  // Measure the depth of the target pixel
  uchar pDepth = pindex(img, x, y);
  if (pDepth == 0) {
    return(false);
  }

  for (iter = offsets.begin(); iter != offsets.end(); ++iter) {

    // Check that our check stays in bounds
    if ((x + iter->x < 0) || 
	(x + iter->x >= width) ||
	(y + iter->y < 0) ||
	(y + iter->y >= height)) {
      return(false);
    }

    // Get the difference between the test pixel's depth (from the
    // camera) and the target pixel's.
    uchar diff = abs(pindex(img, x + iter->x, y + iter->y) - pDepth);

    // Test the kind of comparison we're supposed to be doing.
    if (iter->comp == 0) {

      // The target pixel should be about the same depth as the test
      // pixel.  If not, return false.
      if (diff >= tolerance) {
	return(false);
      }
    } else {
      // The target pixel should be different from the test pixel.
      if (diff < tolerance) {
	return(false);
      }
    }
  }

  // Made it to here = success, the target pixel satisfies the criteria
  // specified with the offsets argument.
  return(true);
};

// Returns the percentage of the given polygon occupied by pixels of
// the value given by depth and displacement.  (displacement = 0, look
// for pixels of depth, disp > 0, look for pixels of depth greater
// than depth, disp < 0, look for pixels closer than depth.  All
// measurements use tolerance.
//
// The plan is eventually to support arbitrary polygons, but for now
// we just operate on rectangles, specified with the lower left and
// upper right corners.
float GTestImgMono::testRect(const int targetDepth, const int tolerance,
			     const CvPoint ll, const CvPoint ur,
			     const double displacement) {
  
  double occupied = 0.0;
  double total = 0.0;
  for (int x = ll.x; x <= ur.x; x++) {

    if (x < 0 || x >= width) continue; 

    for (int y = ll.y; y <= ur.y; y++) { 

      if (y < 0 || y >= height) continue;

      int depth = pindex(img, x, y);
      if (depth == 0) continue;

      total++;
      if (displacement == 0) {
	if (abs(depth - targetDepth) < tolerance) occupied++;
      } else if (displacement < 0) {
	if ((targetDepth - depth) > tolerance/2) occupied++;
      } else {
	if ((depth - targetDepth) > tolerance/2) occupied++;
      }
    }
  }
  occupied = occupied / total;

  return(occupied);
};

  // Moving from (x,y) in the direction implied by (xdir, ydir),
  // returns the coordinates of the first deviation of more than
  // tolerance from the value of (x,y). 
GOffsetPt GTestImgMono::findDiscontinuity(const int x, const int y, 
					  const int tolerance, 
					  const int xdir, 
					  const int ydir) {
  GOffsetPt out(0, 0, 0);
  return(out);
};

IplImage* GTestImgMono::analyzeMotion() {

  if (!motionAnalyzer) {
    motionAnalyzer = new GMotionAnalyzer(img->width, img->height);
  }

  return motionAnalyzer->analyzeMotion(img);

}


// Returns the proportion of pixels indicating horizontal, vertical,
// and oscillatory motion.
GPxHVO GTestImgRGB::testRect(const CvPoint ll, const CvPoint ur) {

  GPxHVO occupied;
  double total = 0.0;
  char* hvo;

  for (int x = ll.x; x <= ur.x; x++) {
    if (x < 0 || x >= width) continue;  

    for (int y = ll.y; y <= ur.y; y++) { 
      if (y < 0 || y >= height) continue;

      hvo = (img->imageData + y * img->widthStep + x * 3);

      total++;

      if ((uchar)hvo[0] == 255) {
	occupied.horiz++; //hl++;
      } else if ((uchar)hvo[0] == 127) {
	occupied.horiz--; //hr++;
      }

      if ((uchar)hvo[1] == 255) {
	occupied.vert++; //vu++;
      } else if ((uchar)hvo[1] == 127) {
	occupied.vert--; //vd++;
      }

      if ((uchar)hvo[2] == 255) occupied.osc++;
    }
  }
  
  occupied.horiz = occupied.horiz / total;
  occupied.vert = occupied.vert / total;
  occupied.osc = occupied.osc / total;

  return(occupied);

}

const int GMotionAnalyzer::offsets[4][2] = {{-1,0},{0,-1},{1,0},{0,1}};

// Glenn suggests subtracting the background -- pixels not at the
// target depth or near it, before doing the image differences.  This
// might make it easier to deal with a moving vehicle.

IplImage* GMotionAnalyzer::analyzeMotion(const IplImage* img) {

  // Create an image to work on.
  IplImage* raw = cvCreateImage(cvSize(img->width, img->height),IPL_DEPTH_8U,1);
  cvResize(img, raw);

  IplImage* diff = NULL;
  if (!moving) {
    diff = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvSub(raw, lastImg, diff);

    // If the difference is greater than threshold, go to 255, 0
    // otherwise.
    cvThreshold(diff, diff, threshold, 255, CV_THRESH_BINARY);
  }

  // Calculate elapsed time interval since last image.
  timeval now;
  gettimeofday(&now, NULL);

  timeval since;
  timersub(&now, &then, &since);
  then = now;

  // Compute elapsed time in milliseconds.
  int elapsed = (since.tv_sec * 1000) + (since.tv_usec / 1000);
  ticks = (ticks + elapsed) % MAXMS;

  // Update 'last'.
  cvCopy(raw, lastImg);

  // If we're moving, there isn't much we can do, so get out.
  if (moving) return(NULL);

  // We seem to be subtracting a fraction of the elapsed milliseconds
  // from traceImg.  This is a hack to get the greys to appear.  If you
  // want more gradations, increase the denominator under 'elapsed'.
  cvSubS(traceImg, cvScalar(elapsed / 4), traceImg);

  // Add the thresholded image to traceImg.
  cvAdd(diff, traceImg, traceImg);

  // Initialize the output image.
  IplImage* outImg = cvCreateImage(cvSize(img->width, img->height),
				IPL_DEPTH_8U, 3);
  cvSetZero(outImg);

  // Initialize some counters.
  int fireCount = 0;

  for (int x = 0; x < width; x++) { 
    for (int y = 0; y < height; y++) {
      uchar levels[4] = {0, 0, 0, 0};


      if (pindex(traceImg, x, y) != 255) continue;

      for (int dir = 0; dir < 4; dir++) {
	for (int i = 0; i < width/4; i++) {
	  int xp = x + i * offsets[dir][0];
	  int yp = y + i * offsets[dir][1];

	  if ((xp < 0) || (xp >= width) || 
	      (yp < 0) || (yp >= height)) break;

	      uchar energy = pindex(traceImg, xp, yp);    

	      if (energy == 255 || energy == 0) continue;

	      levels[dir] = energy;
	      break;
	}
      }

      // If we're here, that means we have a position (x,y) that
      // indicates a full-energy (255) pixel (this is a pixel where
      // the change was greater than threshold), and a four-element
      // array called levels that contains the traces image energy in
      // each of the four neighboring pixels.

      int time = sindex(state, y, x, 0);
      int count = sindex(state, y, x, 1);
      int newState = 1;

      // Red channel is for horizontal motion.
      if (levels[0] > levels[2]) {
	newState = 1;
	pindex(outImg, x * 3, y) = 255; // one direction
      } else if (levels[0] < levels[2]) {
	newState = 2;
	pindex(outImg, x * 3, y) = 127; // the other
      }
      // Green channel for vertical.
      if (levels[1] > levels[3]) {
	pindex(outImg, x * 3 + 1, y) = 255; // one direction
      } else if (levels[1] < levels[3]) {
	pindex(outImg, x * 3 + 1, y) = 127; // the other
      }

      if (newState == 1) {
	sindex(state, y, x, 0) = ticks;
      } else {
	int duration = (ticks - time) % MAXMS;
	if (abs(duration - oscGoal/2) > oscTolerance) {
	  count++;
	} else {
	  count = 0;
	}
      }

      // Blue channel for oscillation.
      if (count >= oscCount) {
	pindex(outImg, x * 3 + 2, y) = 255;
	fireCount++;
	count = 0;
      }

      sindex(state, y, x, 1) = count;

    }
  }

  return(outImg);

}
