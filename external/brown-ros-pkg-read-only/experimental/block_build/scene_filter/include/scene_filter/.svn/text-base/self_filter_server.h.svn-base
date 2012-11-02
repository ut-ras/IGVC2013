/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/** \author Ioan Sucan */

/*
   Modified by Jihoon Lee, Brown University
   Mar 2012
 */

#ifndef _SELF_FILTER_SERVER_H_
#define _SELF_FILTER_SERVER_H_

#include <ros/ros.h>
#include <sstream>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <scene_filter/self_see_filter.h>
#include <scene_filter/IsPointsVisible.h>

namespace scene_filter {

  std::string SELF_FILTER_SRV_NAME = "self_filter_srv";

  template <class PointT>
  class SelfFilterServer
  {
    private :
      ros::NodeHandle                                       nh_, root_handle_;
      ros::Publisher                                        pointCloudPublisher_;
      ros::Subscriber                                       no_filter_sub_;
      ros::ServiceServer                                    filter_srv;

      tf::TransformListener                                 tf_;
      tf::MessageFilter<sensor_msgs::PointCloud2>           *mn_;
      message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_;

      SelfFilter<pcl::PointCloud<PointT> > *self_filter_;
      std::string sensor_frame_;
      double subsample_param_;

      pcl::VoxelGrid<PointT>                         grid_;
    public:

      SelfFilterServer();
      ~SelfFilterServer();

      void noFilterCallback (const sensor_msgs::PointCloud2ConstPtr &cloud);
      void cloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud2);
      bool serviceCallback(IsPointsVisible::Request& req,IsPointsVisible::Response& resp);
  };


  template <class PointT>
  SelfFilterServer<PointT>::SelfFilterServer() : nh_("~") 
  {
    nh_.param<std::string> ("sensor_frame", sensor_frame_, std::string ());
    nh_.param<double> ("subsample_value", subsample_param_, 0.01);
    self_filter_ = new SelfFilter<pcl::PointCloud<PointT> > (nh_);

    sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (root_handle_, "cloud_in", 1);	
    mn_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*sub_, tf_, "", 1);

    //mn_ = new tf::MessageNotifier<sensor_msgs::PointCloud2>(tf_, boost::bind(&SelfFilter::cloudCallback, this, _1), "cloud_in", "", 1);
    pointCloudPublisher_ = root_handle_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
    std::vector<std::string> frames;
    self_filter_->getSelfMask()->getLinkNames(frames);
    if (frames.empty())
    {
      ROS_DEBUG ("No valid frames have been passed into the self filter. Using a callback that will just forward scans on.");
      no_filter_sub_ = root_handle_.subscribe<sensor_msgs::PointCloud2> ("cloud_in", 1, boost::bind(&SelfFilterServer<PointT>::noFilterCallback, this, _1));
    }
    else
    {
      ROS_DEBUG ("Valid frames were passed in. We'll filter them.");
      mn_->setTargetFrames (frames);
      mn_->registerCallback (boost::bind (&SelfFilterServer<PointT>::cloudCallback, this, _1));
    }

    std::string service_name;
    nh_.param<std::string>("filter_srv_name",service_name,SELF_FILTER_SRV_NAME);
    filter_srv = root_handle_.advertiseService<IsPointsVisible::Request,IsPointsVisible::Response>(service_name,boost::bind(&SelfFilterServer<PointT>::serviceCallback,this,_1,_2));
  }

  template <class PointT>
  SelfFilterServer<PointT>::~SelfFilterServer()
  {
    delete mn_;
    delete sub_;
    delete self_filter_;
  }

  template <class PointT>
  void SelfFilterServer<PointT>::noFilterCallback (const sensor_msgs::PointCloud2ConstPtr &cloud)
  {
    pointCloudPublisher_.publish (cloud);
    ROS_DEBUG ("Self filter publishing unfiltered frame");
  }
      
  template <class PointT>
  void SelfFilterServer<PointT>::cloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud2)
  {
    ROS_INFO ("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud2->header.stamp).toSec ());
    std::vector<int> mask;
    ros::WallTime tm = ros::WallTime::now ();

    pcl::PointCloud<PointT> cloud, cloud_filtered;
    pcl::fromROSMsg (*cloud2, cloud);

    if (subsample_param_ != 0)
    {
      pcl::PointCloud<PointT> cloud_downsampled;
      // Set up the downsampling filter
      grid_.setLeafSize (subsample_param_, subsample_param_, subsample_param_);     // 1cm leaf size
      grid_.setInputCloud (boost::make_shared <pcl::PointCloud<PointT> > (cloud));
      grid_.filter (cloud_downsampled);

      self_filter_->updateWithSensorFrame (cloud_downsampled, cloud_filtered, sensor_frame_);
    } 
    else 
    {
      self_filter_->updateWithSensorFrame (cloud, cloud_filtered, sensor_frame_);
    }      

    double sec = (ros::WallTime::now() - tm).toSec ();

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg (cloud_filtered, out);
    pointCloudPublisher_.publish (out);
    ROS_INFO ("Self filter: reduced %d points to %d points in %f seconds", (int)cloud.points.size(), (int)cloud_filtered.points.size (), sec);
  }

  template <class PointT>
  bool SelfFilterServer<PointT>::serviceCallback(IsPointsVisible::Request& req,IsPointsVisible::Response& resp)
  {
    resp.result = self_filter_->getSelfMask()->getMaskIntersection(req.point.x,req.point.y,req.point.z);
    return true;
  }
}

#endif
