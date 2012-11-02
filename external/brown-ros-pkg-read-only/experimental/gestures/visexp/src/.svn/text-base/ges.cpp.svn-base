#include <ros/ros.h>
#include "ges.hpp"
#include <iostream>
#include <stdlib.h>


bool processFrame(IplImage* raw, int ve_width, int ve_height, double m, double b, int ve_tol, int ve_minsize, person_t* person, IplImage* paint_img) {

  static int lastGesture = 0;
  static int streak = 0;
  IplImage* img = cvCreateImage(cvSize(ve_width,ve_height), IPL_DEPTH_8U, 1);
  cvResize(raw,img);

  //start process
  //  IplImage* paint_img = cvCreateImage(cvSize(ve_width,ve_height), IPL_DEPTH_8U, 1);
  cvSetZero(paint_img);

  for (int x = 0; x < ve_width; x++) {
    for (int y = 0; y < ve_height/2; y++) {
      int depth = pindex(img, x, y);
      if (depth == 0) continue;

      //int seg = (int) ((double) ve_sf / (double) (255-depth) + (double) .5);
      int seg = (int) ((m*depth+b) / (double) 2 +(double).5);
      if ( seg <= 0 ) continue;

      //aaa
      if (y < seg/2) continue;
      if (x < seg) continue;

      int aaa = pindex(img, (x-seg), (y-seg/2));
      if (abs(depth - aaa) < ve_tol) continue;

      //bbb
      int bbb = pindex(img, x, (y-seg/2));
      if (abs(depth - bbb) > ve_tol) continue;
      if (bbb == 0) continue;

      //ccc 
      if (x + seg >= ve_width) continue;
      int ccc = pindex(img, x+seg, (y-seg/2));
      if (abs(depth - ccc) < ve_tol) continue;

      //ddd
      if (y + seg/2 >= ve_height) continue;
      int ddd = pindex(img, x-seg, (y+seg/2));
      if (ddd == 0) continue;
      if (abs(depth - ddd) > ve_tol) continue;

      //eee
      int eee = pindex(img, x, (y+seg/2));
      if (eee == 0) continue;
      if (abs(depth - eee) > ve_tol) continue;
      if (abs(eee -ddd ) > ve_tol) continue;

      //fff
      int fff = pindex(img, x+seg, (y+seg/2));
      if (fff == 0) continue;
      if (abs(depth - fff) > ve_tol) continue;
      if (abs(eee -fff) > ve_tol || abs(fff -ddd) > ve_tol) continue;

      //ggg
      if (y + seg >= ve_height) continue;
      int ggg = pindex(img, x, (y+seg));
      if (ggg == 0) continue;
      if (abs(depth - ggg) > ve_tol) continue;
      if (abs(ggg - fff) > ve_tol || abs(ggg -ddd) > ve_tol || abs(ggg-eee) > ve_tol) continue;

      //relationship constraints
      if (abs(aaa - ddd) < ve_tol || abs(ccc - fff) < ve_tol) continue;
      if (abs(bbb - aaa) < ve_tol || abs(bbb - ccc) < ve_tol) continue;


      //hhh
      if (y-seg*2 < 0) continue;
      int hhh = pindex(img, x, (y-seg*2));
      if (abs(hhh - depth) < ve_tol) continue;

      /*
      //d'
      if (x-seg-seg/2 < 0) continue;
      int dp = ((uchar *)(img->imageData + (y+seg/2)*img->widthStep))[x-seg-seg/2];
      if (abs(ddd - dp) < ve_tol) continue;

      //f'
      if (x+seg+seg/2 >= ve_width) continue;
      int fp = ((uchar *)(img->imageData + (y+seg/2)*img->widthStep))[x+seg+seg/2];
      if (abs(fff - fp) < ve_tol) continue;
      */

      pindex(paint_img, x, y) = 255;
      ROS_INFO(">>>>>>> x:%d,y:%d,z:%d",x,y, pindex(paint_img, x, y));

    }
  }

  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* comps;

  cvFindContours(paint_img, storage, &comps, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  if (comps) {
    IplImage* cntr = cvCreateImage(cvSize(ve_width,ve_height), IPL_DEPTH_8U, 3);
    cvSetZero(cntr);
    cvDrawContours(cntr, comps, CV_RGB(255,255,255),CV_RGB(255,0,0),8,1);

    cvCvtColor(cntr, paint_img, CV_RGB2GRAY);


    int ip = 0; 
    for (CvSeq* comp = comps; comp != NULL; comp = comp->h_next) {

      ROS_INFO("%d:%d x %d", ip, ((CvContour *)comp)->rect.width, ((CvContour *)comp)->rect.height);
      if (ip == 0) {
	char name[25];
	sprintf(name, "input%d.jpg", ((CvContour *)comp)->rect.width);
	cvSaveImage(name, raw);
      }
      ip++;
    }

  } else {
    ROS_INFO("NO Contour found");
    int iq = rand() % 10 + 1;
    char name[25];
    sprintf(name, "fail%d.jpg", iq);
    cvSaveImage(name, raw);
  }

  int size = 0;
  CvContour* biggest = NULL;
  for (CvSeq* comp = comps; comp != NULL; comp = comp->h_next) {
    CvContour* contour = (CvContour*) comp;
    int sizeP = contour->rect.width*contour->rect.height;
    if (sizeP < ve_minsize) continue;
    if (sizeP < size) continue;
    size = sizeP;
    biggest = contour;
  }
 
  for (CvContour* contour = biggest; contour != NULL;) { //always true, last line should be break
    int x = contour->rect.x + (contour->rect.width/2);
    int y = contour->rect.y + (contour->rect.height/2);
    
    person->x = x;
    person->y = y;
    person->gesture = 0;

    //repeated by the above
    int depth = pindex(img, x, y);
    //int seg = (int) ((double) ve_sf / (double) (255-depth) + (double) .5);
    int seg = (int) ((m*depth+b) / (double) 2 +(double).5);

    person->rX = contour->rect.x;
    person->rY = contour->rect.y;
    person->rWidth = contour->rect.width;
    person->rHeight = contour->rect.height;
    person->seg = seg;

    double a = 0;
    for (int xp = x-seg*3; xp < x-seg; xp++) {
      for (int yp = y; yp < y+seg; yp++) {
	if (xp < 0 || xp >= ve_width) continue;
	if (yp < 0 || yp >= ve_height) continue;
	
	int depthp = pindex(img, xp, yp);

	if (depthp == 0) continue;
	if (!(depthp > depth-ve_tol) || !(depthp < depth+ve_tol)) continue;

	a++;
      }
    }
    a = a / (seg*2*seg);

    double b = 0;
    for (int xp = x+seg; xp < x+seg*3; xp++) {
      for (int yp = y; yp < y+seg; yp++) {
	if (xp < 0 || xp >= ve_width) continue;
	if (yp < 0 || yp >= ve_height) continue;

	int depthp = pindex(img, xp, yp);

	if (depthp == 0) continue;
	if (!(depthp > depth-ve_tol) || !(depthp < depth+ve_tol)) continue;
	
	b++;
      }
    }
    b = b / (seg*2*seg);

    if (a > .3 && b > .3) {
      person->gesture = 1;
      break;
    }

    if (a > .3 && b < .15) {
      person->gesture = 2;
      break;
    }
    
    if (a < .15 && b > .3) {
      person->gesture = 3;
      break;
    }


    double empty = 0;
    for (int xp = x+seg/2; xp < x+seg; xp++) {
      for (int yp = y - seg; yp < y; yp++) {
	if (xp < 0 || xp >= ve_width) continue;
	if (yp < 0 || yp >= ve_height) continue;

	int depthp = pindex(img, xp, yp);

	if (depthp == 0) continue;
	if (!(depthp > depth-ve_tol) || !(depthp < depth+ve_tol)) continue;

	empty++;
      }
    }
    empty = empty / ((seg/2)*seg);

    double full = 0;
    for (int xp = x+seg; xp < x+seg+seg/2; xp++) {
      for (int yp = y - seg; yp < y; yp++) {
	if (xp < 0 || xp >= ve_width) continue;
	if (yp < 0 || yp >= ve_height) continue;

	int depthp = pindex(img, xp, yp);

	if (depthp == 0) continue;
	if (!(depthp > depth-ve_tol) || !(depthp < depth+ve_tol)) continue;

	full++;
      }
    }
    full = full / ((seg/2)*seg);

    if (full > .30 && empty < .15) {
      person->gesture = 4;
      break;
    }

    break;
    
  }

  streak++;
  if (person->gesture != lastGesture) streak = 0;
  lastGesture = person->gesture;
  person->streak = streak;

  cvReleaseMemStorage(&storage);


  //  cvReleaseImage(&paint_img);
  cvReleaseImage(&img);
  //end process

  return biggest != NULL;
}
