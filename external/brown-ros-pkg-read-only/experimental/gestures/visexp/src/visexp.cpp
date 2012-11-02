#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visexp/Person.h"

#include <complex>
#include <functional>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "ges.hpp"
#include "visexp.hpp"

int bleed = 32;
int energy = 8; 
int hot = 3;

int max_width = 640;
int max_height = 480;

int cur_height = max_height;
int cur_width = max_width;

image_transport::Publisher pub; //used by function below
image_transport::Publisher motion_pub;
image_transport::Publisher dbg_pub;
ros::Publisher person_pub;
ros::Publisher pose_pub;

IplImage* last;
IplImage* motion;

int frame;
void handleFrame(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  // Create and then populate an opencv-appropriate image array.

  // Create...
  cur_height = cloud->height;
  cur_width = cloud->width;
  IplImage* raw = cvCreateImage(cvSize(cur_width,cur_height), IPL_DEPTH_8U, 1);
  cvSetZero(raw);

  // .. then populate.
  {
    int size = cloud->data.size();
    const unsigned char* start = &(cloud->data[0]);
    const unsigned char* cursor = start;

    //ROS_INFO("sz:%d,st:%d,cu:%d,rs:%d,ps:%d",size, start, cursor, cloud->row_step, cloud->point_step);

    int j = 0;
    // ts: changed the following constant from 32 to cloud->point_step 8/9/12
    for (int i = 0; i < size; i += cloud->point_step,j++) {
      cursor = start + i + 8;
      float z = *(reinterpret_cast<const float*>(cursor)); //offset 8
      int zi = (z/15.0)*255.0;
      if (isnan(z)) zi = 0;
      if (zi > 255) zi = 255;
      if (zi < 0) zi = 0;
      ((uchar*)raw->imageData)[j] = zi;
    }
  }
  //ROS_INFO("h:%d,w:%d",cur_height, cur_width);


  person_t person;

  const int ve_width = 500; //500
  const int ve_height = 312; //312

  IplImage* img = cvCreateImage(cvSize(ve_width,ve_height), IPL_DEPTH_8U, 1);
  cvResize(raw,img);

  // Create an output image for monitoring.  This will be a color map
  // indicating the distance from the camera, and we'll use it to
  // superimpose squares and lines that indicate our guesses of where
  // something interesting is going on (for debug purposes).
  IplImage* out_img = cvCreateImage(cvSize(ve_width,ve_height), 
				    IPL_DEPTH_8U, 3);
  cvCvtColor(img,out_img,CV_GRAY2RGB);


  // Populate the basic output image.
  for (int x = 0; x < ve_width; x++) {
    for (int y = 0; y < ve_height; y++) {
      int depth = ((uchar *)(img->imageData + y*img->widthStep))[x];
      if (depth == 0) continue;
      depth = 255 - depth;
      int yStep = y * out_img->widthStep;
      int xChan = x * out_img->nChannels;
      ((uchar *)(out_img->imageData + yStep))[xChan + 2]= gesRed[depth];
      ((uchar *)(out_img->imageData + yStep))[xChan + 1]= gesGreen[depth];
      ((uchar *)(out_img->imageData + yStep))[xChan + 0]= gesBlue[depth];
    }
  }
  // done populating.

  
  // dbg: ts.
  IplImage* paint_img = cvCreateImage(cvSize(ve_width,ve_height), IPL_DEPTH_8U, 1);
//was -1,124,4,50
  if (processFrame(raw, ve_width, ve_height, -1.5, 124, 4, 50, &person, paint_img)) {
    CvScalar color = cvScalar(255,255,255);
    if (person.streak > hot) {
      if (person.gesture == 1) color = cvScalar(0,204,0);
      if (person.gesture == 2) color = cvScalar(153,153,0);
      if (person.gesture == 3) color = cvScalar(0,116,255);
      if (person.gesture == 4) color = cvScalar(255,0,0);
      if (person.gesture == 5) color = cvScalar(0,0,255);
      if (person.gesture == 6) color = cvScalar(0,0,0);
    }

    cvRectangle(out_img, 
		cvPoint(person.rX,person.rY), 
		cvPoint(person.rX+person.rWidth,person.rY+person.rHeight), 
		color, -1);

    int x = person.x;
    int y = person.y;
    int seg = person.seg;

    color = cvScalar(0,0,255);
    cvLine(out_img, cvPoint(x-seg,y-seg/2), cvPoint(x+seg,y-seg/2), color, 1);
    cvLine(out_img, cvPoint(x-seg,y+seg/2), cvPoint(x+seg,y+seg/2), color, 1);
    cvLine(out_img, cvPoint(x,y), cvPoint(x,y+seg), cvScalar(255,0,0),1);

    // Draw two rectangles around presumed hand points?
    cvRectangle(out_img, cvPoint(x-seg*3,y), cvPoint(x-seg,y+seg), color, 5);
    cvRectangle(out_img, cvPoint(x+seg,y), cvPoint(x+seg*3,y+seg), color, 5);

    //if (person.streak > hot) { //org
    {
      visexp::Person target;
      target.streak = person.streak;
      target.gesture = person.gesture;
      if (person.streak <= hot) person.gesture = 0;

      geometry_msgs::PointStamped point;
      geometry_msgs::PoseStamped pose;
      pose.header = cloud->header;
      point.header = cloud->header;
      int min_dis = 1000000;
      int imageDepth = 0;
      int size = cloud->data.size();
      int x = 0;
      int y = 0;
      int j = 0;
      const unsigned char* start = &(cloud->data[0]);
      const unsigned char* cursor = start;
      for (int i = 0; i < size; i += 32, x++,j++) {
        if (x >= cur_width) {
          x = 0;
          y++;
        }
        y = y % cur_height;

        //based on shrinkage
	int dis = (1.28*person.x-x)*(1.28*person.x-x)+(1.53*person.y-y)*(1.53*person.y-y);

	if (dis < min_dis) {
          cursor = start + i; //32 for each point rgb data is at offset 16

          float space = *(reinterpret_cast<const float*>(cursor)); //offset 0
          if (isnan(space)) {
            continue;
          }

	  min_dis = dis;

          point.point.x = *(reinterpret_cast<const float*>(cursor)); //offset 0
          cursor += 4;
          point.point.y = *(reinterpret_cast<const float*>(cursor)); //offset 4
          cursor += 4;
          point.point.z = *(reinterpret_cast<const float*>(cursor)); //offset 8

          imageDepth = ((uchar*)raw->imageData)[j];

	}
      }

      {
        int size = cur_width*cur_height;
        for (int i = 0; i < size; i++) {
          int now = abs(((uchar*)raw->imageData)[i] - imageDepth);
          if (now > 255) now = 255;
          if (now < 0) now = 0;
          int then = ((uchar*)last->imageData)[i];
          ((uchar*)last->imageData)[i] = now;
          int amt = abs(now-then);
          int mot = ((uchar*)motion->imageData)[i] + amt*energy - bleed;
          if (mot > 255) mot = 255;
          if (mot < 0) mot = 0;
          ((uchar*)motion->imageData)[i] = mot;
        }
      }

      target.position = point;
      pose.pose.position = point.point;
      person_pub.publish(target);
      pose_pub.publish(pose);
    }
  }

  // dbg: ts.
  sensor_msgs::ImagePtr dbg = sensor_msgs::CvBridge::cvToImgMsg(paint_img, "mono8");
  dbg_pub.publish(dbg);
  cvReleaseImage(&paint_img);

	/*
	std::stringstream filename;
	filename << "movie/" << std::setw(4) << std::setfill('0') << frame++ << ".png";
	const std::string& tmp = filename.str();
	const char* filenameCstr = tmp.c_str();
	cvSaveImage(filenameCstr, raw);
	*/

  //  sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(raw, "mono8");
  sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(out_img, "rgb8");
  pub.publish(out);

  sensor_msgs::ImagePtr out2 = sensor_msgs::CvBridge::cvToImgMsg(motion, "mono8");
  motion_pub.publish(out2);
  cvReleaseImage(&out_img);
  cvReleaseImage(&img);
  cvReleaseImage(&raw);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visexp");
  ros::NodeHandle nh;

  frame = 0;

  image_transport::ImageTransport it(nh);

  pub = it.advertise("visexp/image", 1);
  motion_pub = it.advertise("visexp/motion", 1);
  dbg_pub = it.advertise("visexp/dbgimg", 1);

  last = cvCreateImage(cvSize(cur_width,cur_height), IPL_DEPTH_8U, 1);
  motion = cvCreateImage(cvSize(cur_width,cur_height), IPL_DEPTH_8U, 1);
  cvSetZero(motion);

  ros::Subscriber sub = nh.subscribe("points", 1, handleFrame);

  person_pub = nh.advertise<visexp::Person>("visexp/person",1000);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("visexp/pose",1000);

  ros::spin();

  cvReleaseImage(&last);
  cvReleaseImage(&motion);
}
