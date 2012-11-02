/*
   Object Tracker

  Author : Jihoon
  Date : 12.2011
 */

#ifndef _ObjectTracker_H_
#define _ObjectTracker_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <object_tracker/Blobgroup.h>
#include <image_transport/image_transport.h>
#include "kalman_filter.h"

struct GroupKF {
  GroupKF(unsigned int n,unsigned int s, KalmanFilter k) { gNum = n; kf = k; }
  unsigned int gNum;
  unsigned int size;
  KalmanFilter kf;
};

class ObjectTracker {
  public:
    ObjectTracker(ros::NodeHandle& nh);
    ~ObjectTracker();

    void spin();
    void msgCallback(const object_tracker::Blobgroup& msg);
    void getMatchingBlob(geometry_msgs::Pose oldp,std::vector<geometry_msgs::Pose> newp,int& idx,bool chk[]);
    void publishObjects();

    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
  private:
    ros::NodeHandle node;
 
    ros::Publisher pub;
    image_transport::Publisher pub2;
    ros::Subscriber sub;
  image_transport::Subscriber sub2;
    image_transport::ImageTransport it;

    std::string subtopic;
    std::string pubtopic;

    std::vector<GroupKF> gkf;
    ros::Time t;
    unsigned int objNum;
};

#endif // _ObjectTracker_H_
