/*
  BackgroundModeler
  
  it susbscribes given image stream topic and saves gaussian distribution of each pixel

  Author : Jihoon Lee
  Date   : 11.2011
 */

#ifndef _BGMODELER_H_
#define _BGMODELER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include "bgmodel.h"

class BGModeler
{
  public:
    BGModeler(ros::NodeHandle& node,int numSet);
    ~BGModeler();
    void spin();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  private:
    ros::NodeHandle node;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    std::string topic;
    int numSet; 
    int cnt;
    bool isFirst;
    bool isReady;
    std::string bgm_out;
    BGModel* bgm;
};

#endif // _BGMODELER_H_
