/*
 * Copyright (C) 2009
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 */

#include <boost/thread/mutex.hpp>

#include "ros/ros.h"

// messages
#include <amtec/AmtecState.h>

// services
#include <amtec/GetStatus.h>
#include <amtec/Halt.h>
#include <amtec/Home.h>
#include <amtec/Reset.h>
#include <amtec/SetPosition.h>
#include <amtec/SetVelocity.h>
#include <amtec/TargetAcceleration.h>
#include <amtec/TargetVelocity.h>

#define PT_AT_POINT_DIST        0.15

class SweepAmtec
{
public:
  ros::NodeHandle node_;

  // subscribers
  ros::Subscriber amtec_pan_state_sub_;
  ros::Subscriber amtec_tilt_state_sub_;

  // amtec state
  amtec::AmtecState amtec_pan_state_;
  amtec::AmtecState amtec_tilt_state_;
  boost::mutex amtec_state_mutex_;

  double pan_start_;
  double pan_stop_;
  double pan_accel_;
  double pan_vel_;

  SweepAmtec()
  {
    amtec_pan_state_sub_ = node_.subscribe("/amtec/pan_state", 10, &SweepAmtec::amtecPanState, this);
    amtec_tilt_state_sub_ = node_.subscribe("/amtec/tilt_state", 10, &SweepAmtec::amtecTiltState, this);

    ros::NodeHandle private_ns("~");
    private_ns.param("pan_start", pan_start_, -0.6);
    private_ns.param("pan_stop", pan_stop_,  0.6);
    private_ns.param("pan_acceleration", pan_accel_, 4.0);
    private_ns.param("pan_velocity", pan_vel_, 2.5);
  }

  virtual ~SweepAmtec()
  {

  }

  void amtecPanState(const amtec::AmtecStateConstPtr& in)
  {
    amtec_state_mutex_.lock();
    amtec_pan_state_ = *in;
    amtec_state_mutex_.unlock();
  }

  void amtecTiltState(const amtec::AmtecStateConstPtr& in)
  {
    amtec_state_mutex_.lock();
    amtec_tilt_state_ = *in;
    amtec_state_mutex_.unlock();
  }

  bool amtecSetTargetVelocity(double velocity_pan, double velocity_tilt)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<amtec::TargetVelocity>("/amtec/target_velocity");
    amtec::TargetVelocity srv;
    srv.request.velocity_pan = velocity_pan;
    srv.request.velocity_tilt = velocity_tilt;
    return client.call(srv);
  }

  bool amtecSetTargetAcceleration(double acceleration_pan, double acceleration_tilt)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<amtec::TargetAcceleration>("/amtec/target_acceleration");
    amtec::TargetAcceleration srv;
    srv.request.acceleration_pan = acceleration_pan;
    srv.request.acceleration_tilt = acceleration_tilt;
    return client.call(srv);
  }

  bool amtecSetPosition(double pan, double tilt)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<amtec::SetPosition>("/amtec/set_position");
    amtec::SetPosition srv;

    while (pan > 190.0)
      pan -= 360.0;
    while (pan < -190.0)
      pan += 360.0;

    srv.request.position_pan = pan;
    srv.request.position_tilt = tilt;
    return client.call(srv);
  }

  bool sweep()
  {
    ROS_INFO("set target velocity %f %f", pan_vel_, 0.1);
    amtecSetTargetVelocity(pan_vel_, 0.1);


    amtecSetTargetAcceleration(pan_accel_, 0.5);
    amtecSetTargetVelocity(pan_vel_, 0.1);
    int scan_state = 0;
    double pan_target = pan_start_;
    amtecSetPosition(pan_target, 0.0);
    ROS_INFO("sweeping to start....");

    ros::Rate r(100);
    while(node_.ok())
    {
      amtec_state_mutex_.lock();
      double current_pan = amtec_pan_state_.position;
      amtec_state_mutex_.unlock();

      double pan_target_wrapped = pan_target;
      while (pan_target_wrapped > M_PI)
        pan_target_wrapped -= 2*M_PI;
      while (pan_target_wrapped < -M_PI)
        pan_target_wrapped += 2*M_PI;

      double pan_dist = fabs(pan_target_wrapped - current_pan);
      //fprintf(stderr, "p=%f t=%f d=%f (pd=%f td=%f)\n", current_pan, current_tilt, dist, pan_dist, tilt_dist);
      switch (scan_state) {
      case 0: // moving to left position
      {
        if (pan_dist < PT_AT_POINT_DIST) {
          scan_state = 1;
          amtecSetTargetVelocity(pan_vel_, 0.1);
          pan_target = pan_stop_;
          amtecSetPosition(pan_target, 0.0);
          ROS_DEBUG("sweeping to stop....");
        }
        break;
      }
      case 1: // moving to left position
      {
        if (pan_dist < PT_AT_POINT_DIST) {
          scan_state = 0;
          amtecSetTargetVelocity(pan_vel_, 0.1);
          pan_target = pan_start_;
          amtecSetPosition(pan_target, 0.0);
          ROS_DEBUG("sweeping to start....");
        }
        break;
      }
      }

      // spin once to allow callbacks
      ros::spinOnce();
      r.sleep();
    }

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sweep");
  SweepAmtec s;
  s.sweep();
  return 0;
}
