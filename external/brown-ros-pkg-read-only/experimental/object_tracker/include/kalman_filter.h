/*
  Kalman filter for object tracking

  Author : Jihoon
  Date : 12.2011
 */

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include <ros/ros.h>
#include <object_tracker/Blobgroup.h>
#include <geometry_msgs/Pose.h>

#include "matrix2x2.h"

class KalmanFilter {
  public:
    KalmanFilter();
    KalmanFilter(float x,float y);  // init pose and vel
    ~KalmanFilter();

    void update(object_tracker::Blobgroup group,float dt);
    void predict(float dt);
    void correct(float nx,float ny);
    object_tracker::Blobgroup getNewState();
    geometry_msgs::Pose getPose();

  private:

    float sig_a;
    ros::Time t;
    Matrix2x2 pre_cov;
    Matrix2x2 post_cov;
    Matrix2x2 pre_pose;
    Matrix2x2 post_pose;
    Float2 H;
};

#endif // Kalman Filter
