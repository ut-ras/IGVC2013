/*
  Object Detector

  Author : Jihoon
  Date : 12.2011
*/

#ifndef _ObjectDetector_H_
#define _ObjectDetector_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <object_tracker/Blobgroup.h>

#include "bgmodel.h"
#include "morpho.h"
#include "group.h"

class ObjectDetector {
  public:
    ObjectDetector(ros::NodeHandle& node);
    ~ObjectDetector();
    void spin();
    void fgCallback(const sensor_msgs::ImageConstPtr& msg);
    void subtractBackground(sensor_msgs::Image& newimg,bool bimg[],const sensor_msgs::ImageConstPtr& msg);
    void grouping(bool bimg[],int g[],int w,int h);
    void groupit(bool img[],Group& group,int g[],int y,int x,int w,int h,int groupNum);
    void publishBlobgroup();

    void output(char* filename,int* img,int w,int h);
    void convertAndPub(sensor_msgs::Image& newimg,const sensor_msgs::ImageConstPtr& msg,int* gr);

  private:
    ros::NodeHandle node;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    image_transport::Publisher pub2;
    ros::Publisher pub3;

    std::string bgmodel_file;
    std::string topic;
    std::string pubtopic; // debugging purpose
    BGModel bg;
    Morpho morpho;
    std::vector<Group> group;
};

#endif // _ObjectDetector_H_
