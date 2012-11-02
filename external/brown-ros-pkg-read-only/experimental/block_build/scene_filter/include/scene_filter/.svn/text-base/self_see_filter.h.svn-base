/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
   Modified by Jihoon Lee,Brown University

   splitted to .h and .cpp and some more functions are added

   Mar 2012
 */

#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <filters/filter_base.h>
#include <scene_filter/self_mask.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace scene_filter
{
  /** \brief A filter to remove parts of the robot seen in a pointcloud */
  template <typename PointCloud>
  class SelfFilter: public filters::FilterBase <PointCloud>
  {
    public:
      /** \brief Construct the filter */
      SelfFilter (ros::NodeHandle nh);
      ~SelfFilter(void);
        
      bool configure (void);

      bool updateWithSensorFrame(const PointCloud& data_in, PointCloud& data_out, const std::string& sensor_frame);
        
      /** \brief Update the filter and return the data seperately
       * \param data_in T array with length width
       * \param data_out T array with length width
       */
      bool update (const PointCloud& data_in, PointCloud& data_out);

      bool updateWithSensorFrame (const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff, const std::string& sensor_frame);

      /** \brief Update the filter and return the data seperately
       * \param data_in T array with length width
       * \param data_out T array with length width
       */
      bool update (const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff);

      void fillDiff (const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out);

      void fillResult (const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out);

      SelfMask<PointCloud>* getSelfMask ();

      void setSensorFrame (const std::string& frame); 
        
    protected:
        
      tf::TransformListener tf_;
      SelfMask<PointCloud>* sm_;
      
      ros::NodeHandle nh_;
      bool invert_;
      std::string sensor_frame_;
      std::string annotate_;
      double min_sensor_dist_;
  };


  /** \brief Construct the filter */
  template<typename PointCloud>
  SelfFilter<PointCloud>::SelfFilter (ros::NodeHandle nh) : nh_(nh)
  {
    nh_.param<double> ("min_sensor_dist", min_sensor_dist_, 0.01);
    double default_padding, default_scale;
    nh_.param<double> ("self_see_default_padding", default_padding, .01);
    nh_.param<double> ("self_see_default_scale", default_scale, 1.0);
    
    std::vector<scene_filter::LinkInfo> links;	
    std::string link_names;
    
    if (!nh_.hasParam ("self_see_links")) 
    {
      ROS_WARN ("No links specified for self filtering.");
    } 
    else 
    {     
      XmlRpc::XmlRpcValue ssl_vals;;
      
      nh_.getParam ("self_see_links", ssl_vals);
      if (ssl_vals.getType () != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN ("Self see links need to be an array");
      } 
      else 
      {
        if (ssl_vals.size () == 0) 
        {
          ROS_WARN ("No values in self see links array");
        } 
        else 
        {
          for (int i = 0; i < ssl_vals.size (); ++i)
          {
            scene_filter::LinkInfo li;
            
            if (ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) 
            {
              ROS_WARN ("Self see links entry %d is not a structure.  Stopping processing of self see links", i);
              break;
            }
            if (!ssl_vals[i].hasMember ("name")) 
            {
              ROS_WARN ("Self see links entry %d has no name.  Stopping processing of self see links", i);
              break;
            } 
            li.name = std::string (ssl_vals[i]["name"]);
            if (!ssl_vals[i].hasMember ("padding")) 
            {
              ROS_DEBUG ("Self see links entry %d has no padding.  Assuming default padding of %g", i, default_padding);
              li.padding = default_padding;
            } 
            else 
            {
              li.padding = ssl_vals[i]["padding"];
            }
            if (!ssl_vals[i].hasMember ("scale")) 
            {
              ROS_DEBUG ("Self see links entry %d has no scale.  Assuming default scale of %g", i, default_scale);
              li.scale = default_scale;
            } 
            else 
            {
              li.scale = ssl_vals[i]["scale"];
            }
            links.push_back (li);
          }
        }
      }
    }
    sm_ = new scene_filter::SelfMask<PointCloud> (tf_, links);
    nh_.param<std::string> ("annotate", annotate_, std::string ());

    if (!annotate_.empty ())
        ROS_INFO ("Self filter is adding annotation channel '%s'", annotate_.c_str ());
    if (!sensor_frame_.empty ())
      ROS_INFO("Self filter is removing shadow points for sensor in frame '%s'. Minimum distance to sensor is %f.", sensor_frame_.c_str (), min_sensor_dist_);
  }

  template<typename PointCloud>
  SelfFilter<PointCloud>::~SelfFilter (void)
  {
    delete sm_;
  }

  template<typename PointCloud>
  bool SelfFilter<PointCloud>::configure (void)
  {
    // keep only the points that are outside of the robot
    // for testing purposes this may be changed to true
    nh_.param ("invert", invert_, false);
    
    if (invert_)
      ROS_INFO ("Inverting filter output");
  
    return (true);
  }

  template<typename PointCloud>
  bool SelfFilter<PointCloud>::updateWithSensorFrame(const PointCloud& data_in, PointCloud& data_out, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update (data_in, data_out);
  }

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  template<typename PointCloud>
  bool SelfFilter<PointCloud>::update (const PointCloud& data_in, PointCloud& data_out)
  {
    std::vector<int> keep (data_in.points.size ());
    if (sensor_frame_.empty ()) 
    {
      sm_->maskContainment (data_in, keep);
    } 
    else 
    {
      sm_->maskIntersection (data_in, sensor_frame_, min_sensor_dist_, keep);
    }	
    fillResult (data_in, keep, data_out);
    return (true);
  }

  template<typename PointCloud>
  bool SelfFilter<PointCloud>::updateWithSensorFrame (const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return (update (data_in, data_out, data_diff));
  }

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  template<typename PointCloud>
  bool SelfFilter<PointCloud>::update (const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff)
  {
    std::vector<int> keep (data_in.points.size ());
    if (sensor_frame_.empty ()) 
    {
      sm_->maskContainment (data_in, keep);
    } 
    else 
    {
      sm_->maskIntersection (data_in, sensor_frame_, min_sensor_dist_, keep);
    }
    fillResult (data_in, keep, data_out);
    fillDiff (data_in,keep,data_diff);
    return (true);
  }

  template<typename PointCloud>                                                                 
  void SelfFilter<PointCloud>::fillDiff (const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out)
  {
    const unsigned int np = data_in.points.size ();
  
    // fill in output data 
    data_out.header = data_in.header;	  
  
    data_out.points.resize (0);
    data_out.points.reserve (np);
  
    for (unsigned int i = 0 ; i < np ; ++i)
    {
      if ((keep[i] && invert_) || (!keep[i] && !invert_))
      {
        data_out.points.push_back(data_in.points[i]);
      }
    }
  }

  template<typename PointCloud>
  void SelfFilter<PointCloud>::fillResult (const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out)
  {
    const unsigned int np = data_in.points.size ();

    // fill in output data with points that are NOT on the robot
    data_out.header = data_in.header;	  
  
    data_out.points.resize (0);
    data_out.points.reserve (np);
  
    for (unsigned int i = 0 ; i < np ; ++i)
    {
      if (keep[i] == scene_filter::OUTSIDE)
      {
        data_out.points.push_back (data_in.points[i]);
      }
    }
  }

  template<typename PointCloud>
  SelfMask<PointCloud>* SelfFilter<PointCloud>::getSelfMask ()
  {
    return (sm_);
  }

  template<typename PointCloud>
  void SelfFilter<PointCloud>::setSensorFrame (const std::string& frame) 
  {
    sensor_frame_ = frame;
  }
}

#endif
