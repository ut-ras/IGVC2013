/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *
 *********************************************************************/

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <amtec_base.h>
#include <amtec_commands.h>
#include <amtec_settings.h>
#include <amtec_io.h>

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
#include <amtec/SweepPan.h>
#include <amtec/SweepTilt.h>

class AmtecNode
{
public:
  ros::NodeHandle node_;
  boost::mutex amtec_mutex_ ;
  amtec_powercube_p amtec_;
  amtec::AmtecState amtec_pan_state_message_;
  amtec::AmtecState amtec_tilt_state_message_;

  ros::Publisher pan_state_pub_;
  ros::Publisher tilt_state_pub_;

  ros::ServiceServer get_status_srv_;
  ros::ServiceServer halt_srv_;
  ros::ServiceServer home_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer set_position_srv_;
  ros::ServiceServer set_velocity_srv_;
  ros::ServiceServer target_accel_srv_;
  ros::ServiceServer target_vel_srv_;
  ros::ServiceServer sweep_pan_srv_;
  ros::ServiceServer sweep_tilt_srv_;

  tf::TransformBroadcaster tf_;
  tf::Transform parent_to_amtec_;
  std::string parent_frame_;
  std::string amtec_frame_;

  // parameter
  AmtecNode() : node_(), amtec_(NULL), tf_()
  {
    std::string device, parity;

    // initialize amtec
    ROS_INFO("Initializing Amtec");
    amtec_ = amtecInitialize();

    parent_to_amtec_.setIdentity();

    ros::NodeHandle private_nh("~");
    // ***** Parameters *****
    private_nh.param("parent_frame", parent_frame_, std::string("head_link"));
    private_nh.param("amtec_frame", amtec_frame_, std::string("amtec_link"));

    private_nh.param("port", device, std::string("/dev/ttyUSB2"));
    private_nh.param("baud_rate", amtec_->dev.baud, 38400);
    private_nh.param("data_bits", amtec_->dev.databits, 8);
    private_nh.param("parity", parity, std::string("N"));
    private_nh.param("stop_bits", amtec_->dev.stopbits, 1);
    private_nh.param("hardware_flow_control", amtec_->dev.hwf, 0);
    private_nh.param("software_flow_control", amtec_->dev.swf, 0);

    private_nh.param("pan_id", amtec_->pan.id, 14);
    private_nh.param("pan_min_pos", amtec_->panset.MinPos, -3.5);
    private_nh.param("pan_max_pos", amtec_->panset.MaxPos, 3.5);
    private_nh.param("pan_max_vel", amtec_->panset.MaxVel, 2.0);
    private_nh.param("pan_max_acc", amtec_->panset.MaxAcc, 5.0);
    private_nh.param("pan_max_cur", amtec_->panset.MaxCur, 10.0);
    private_nh.param("pan_c0", amtec_->panset.C0, 12);
    private_nh.param("pan_damp", amtec_->panset.Damp, 4);
    private_nh.param("pan_a0", amtec_->panset.A0, 2);

    private_nh.param("tilt_id", amtec_->tilt.id, 13);
    private_nh.param("tilt_min_pos", amtec_->tiltset.MinPos, -1.57);
    private_nh.param("tilt_max_pos", amtec_->tiltset.MaxPos, 1.57);
    private_nh.param("tilt_max_vel", amtec_->tiltset.MaxVel, 1.0);
    private_nh.param("tilt_max_acc", amtec_->tiltset.MaxAcc, 2.5);
    private_nh.param("tilt_max_cur", amtec_->tiltset.MaxCur, 10.0);
    private_nh.param("tilt_use_break", amtec_->tiltset.UseBreak, 1);
    private_nh.param("tilt_c0", amtec_->tiltset.C0, 18);
    private_nh.param("tilt_damp", amtec_->tiltset.Damp, 3);
    private_nh.param("tilt_a0", amtec_->tiltset.A0, 1);

    strcpy(amtec_->dev.ttyport, device.c_str());
    if (parity != "") {
      if (parity == "N")
        amtec_->dev.parity = N;
      else if (parity == "E")
        amtec_->dev.parity = E;
      else if (parity == "O")
        amtec_->dev.parity = O;
      else {
        ROS_FATAL("ERROR: unknown PARITY_TYPE %s", parity.c_str());
        ROS_FATAL("       valid types: N, E, O");
        private_nh.shutdown();
        return;
      }
    }

    // connect amtec
    ROS_INFO("Connecting to amtec to %s", amtec_->dev.ttyport);
    if (amtecDeviceConnectPort(&amtec_->dev) < 0) {
      ROS_FATAL("Unable to connect amtec at %s", amtec_->dev.ttyport);
      private_nh.shutdown();
      return;
    }

    ROS_INFO("Resetting device");
    if(!amtecReset(&amtec_->dev, amtec_->pan.id)) {
      ROS_FATAL("Unable to connect to pan module");
      private_nh.shutdown();
      return;
    }
    if(!amtecReset(&amtec_->dev, amtec_->tilt.id)) {
      ROS_FATAL("Unable to connect tilt module");
      private_nh.shutdown();
      return;
    }

    ROS_INFO("Retrieving module state");
    unsigned int pan_serial = amtecGetDefCubeSerial(&amtec_->dev, amtec_->pan.id);
    unsigned int pan_state = amtecGetCubeState(&amtec_->dev, amtec_->pan.id);
    unsigned int tilt_serial = amtecGetDefCubeSerial(&amtec_->dev, amtec_->tilt.id);
    unsigned int tilt_state = amtecGetCubeState(&amtec_->dev, amtec_->pan.id);

    std::cout << "pan serial " << pan_serial << std::endl;
    std::cout << "pan state: " << std::endl;
    printModuleState(pan_state);

    std::cout << "tilt serial " << tilt_serial << std::endl;
    std::cout << "tilt state: " << std::endl;
    printModuleState(tilt_state);

    ROS_INFO("Homing device");
    amtecHome(&amtec_->dev, amtec_->pan.id);
    amtecHome(&amtec_->dev, amtec_->tilt.id);
    sleep(3);

    ROS_INFO("set PAN and TILT settings");
    amtecSetSettings(amtec_);

    ROS_INFO("Loading parameters");
    amtecGetParams(amtec_);

    // ***** Advertise Messages *****
    pan_state_pub_ = private_nh.advertise<amtec::AmtecState>("pan_state", 1);
    tilt_state_pub_ = private_nh.advertise<amtec::AmtecState>("tilt_state", 1);

    // ***** Start Services *****
    get_status_srv_ = private_nh.advertiseService("get_status", &AmtecNode::getStatus, this);
    halt_srv_ = private_nh.advertiseService("halt", &AmtecNode::halt, this);
    home_srv_ = private_nh.advertiseService("home", &AmtecNode::home, this);
    reset_srv_ = private_nh.advertiseService("reset", &AmtecNode::reset, this);
    set_position_srv_ = private_nh.advertiseService("set_position", &AmtecNode::setPosition, this);
    set_velocity_srv_ = private_nh.advertiseService("set_velocity", &AmtecNode::setVelocity, this);
    target_accel_srv_ = private_nh.advertiseService("target_acceleration", &AmtecNode::targetAcceleration, this);
    target_vel_srv_ = private_nh.advertiseService("target_velocity", &AmtecNode::targetVelocity, this);
    sweep_pan_srv_ = private_nh.advertiseService("sweep_pan", &AmtecNode::sweepPan, this);
    sweep_tilt_srv_ = private_nh.advertiseService("sweep_tilt", &AmtecNode::sweepTilt, this);
    ROS_INFO("Amtec is ready!");
  }

  virtual ~AmtecNode()
  {
    // shutdown amtec
    amtecClear(amtec_);
  }

  void printModuleState(unsigned int state)
  {
    if(state&STATE_HOME_OK) std::cout << "STATE_HOME_OK" << std::endl;
    if(state&STATE_HALTED) std::cout << "STATE_HALTED" << std::endl;
    if(state&STATE_SWR) std::cout << "STATE_SWR" << std::endl;
    if(state&STATE_SW1) std::cout << "STATE_SW1" << std::endl;
    if(state&STATE_SW2) std::cout << "STATE_SW2" << std::endl;
    if(state&STATE_BRAKEACTIVE) std::cout << "STATE_BRAKEACTIVE" << std::endl;
    if(state&STATE_CURLIMIT) std::cout << "STATE_CURLIMIT" << std::endl;
    if(state&STATE_MOTION) std::cout << "STATE_MOTION" << std::endl;
    if(state&STATE_RAMP_ACC) std::cout << "STATE_RAMP_ACC" << std::endl;
    if(state&STATE_RAMP_STEADY) std::cout << "STATE_RAMP_STEADY" << std::endl;
    if(state&STATE_RAMP_DEC) std::cout << "STATE_RAMP_DEC" << std::endl;
    if(state&STATE_RAMP_END) std::cout << "STATE_RAMP_END" << std::endl;
    if(state&STATE_INPROGRESS) std::cout << "STATE_INPROGRESS" << std::endl;
    if(state&STATE_FULLBUFFER) std::cout << "STATE_FULLBUFFER" << std::endl;
    if(state&STATE_ERROR) std::cout << "STATE_ERROR" << std::endl;
    if(state&STATE_POWERFAULT) std::cout << "STATE_POWERFAULT" << std::endl;
    if(state&STATE_TOW_ERROR) std::cout << "STATE_TOW_ERROR" << std::endl;
    if(state&STATE_COMM_ERROR) std::cout << "STATE_COMM_ERROR" << std::endl;
    if(state&STATE_POW_VOLT_ERR) std::cout << "STATE_POW_VOLT_ERR" << std::endl;
    if(state&STATE_POW_FET_TEMP) std::cout << "STATE_POW_FET_TEMP" << std::endl;
    if(state&STATE_POW_INTEGRALERR) std::cout << "STATE_POW_INTEGRALERR" << std::endl;
    if(state&STATE_BEYOND_HARD) std::cout << "STATE_BEYOND_HARD" << std::endl;
    if(state&STATE_BEYOND_SOFT) std::cout << "STATE_BEYOND_SOFT" << std::endl;
    if(state&STATE_LOGIC_VOLT) std::cout << "STATE_LOGIC_VOLT" << std::endl;
  }

  bool getStatus(amtec::GetStatus::Request& req, amtec::GetStatus::Response& resp)
  {
    amtec_mutex_.lock();
    resp.position_pan = -amtecGetActPos(&amtec_->dev, amtec_->pan.id);
    resp.position_tilt = amtecGetActPos(&amtec_->dev, amtec_->tilt.id);
    resp.velocity_pan = -amtecGetActVel(&amtec_->dev, amtec_->pan.id);
    resp.velocity_tilt = amtecGetActVel(&amtec_->dev, amtec_->tilt.id);
    amtec_mutex_.unlock();
    return true;
  }

  bool halt(amtec::Halt::Request& req, amtec::Halt::Response& resp)
  {
    bool ret = true;
    amtec_mutex_.lock();
    ret = ret && amtecHalt(&amtec_->dev, amtec_->pan.id);
    ret = ret && amtecHalt(&amtec_->dev, amtec_->tilt.id);
    amtec_mutex_.unlock();
    return ret;
  }

  bool home(amtec::Home::Request& req, amtec::Home::Response& resp)
  {
    bool ret;
    amtec_mutex_.lock();
    ret = amtecHome(&amtec_->dev, amtec_->pan.id);
    ret = ret && amtecHome(&amtec_->dev, amtec_->tilt.id);
    amtec_mutex_.unlock();
    return ret;
  }

  bool reset(amtec::Reset::Request& req, amtec::Reset::Response& resp)
  {
    bool ret;
    amtec_mutex_.lock();
    ret = amtecReset(&amtec_->dev, amtec_->pan.id);
    ret = ret && amtecReset(&amtec_->dev, amtec_->tilt.id);
    amtec_mutex_.unlock();
    return ret;
  }

  bool setPosition(amtec::SetPosition::Request& req, amtec::SetPosition::Response& resp)
  {
    bool ret;
    amtec_mutex_.lock();
    ret = amtecMotionFRamp(&amtec_->dev, amtec_->pan.id, -req.position_pan);

    // to avoid gratuitous locking if already at requested tilt position
    if (fabs(req.position_tilt - amtec_tilt_state_message_.position) > 0.001)
      ret = ret && amtecMotionFRamp(&amtec_->dev, amtec_->tilt.id, req.position_tilt);

    amtec_mutex_.unlock();
    return ret;
  }

  bool setVelocity(amtec::SetVelocity::Request& req, amtec::SetVelocity::Response& resp)
  {
    bool ret;
    amtec_mutex_.lock();
    ret = amtecMotionFVel(&amtec_->dev, amtec_->pan.id, -req.velocity_pan);
    ret = ret && amtecMotionFVel(&amtec_->dev, amtec_->tilt.id, req.velocity_tilt);
    amtec_mutex_.unlock();
    return ret;
  }

  bool targetAcceleration(amtec::TargetAcceleration::Request& req, amtec::TargetAcceleration::Response& resp)
  {
    bool ret = true;
    amtec_mutex_.lock();
    amtecSetTargetAcc(&amtec_->dev, amtec_->pan.id, req.acceleration_pan);
    amtecSetTargetAcc(&amtec_->dev, amtec_->tilt.id, req.acceleration_tilt);
    amtec_mutex_.unlock();
    return ret;
  }

  bool targetVelocity(amtec::TargetVelocity::Request& req, amtec::TargetVelocity::Response& resp)
  {
    bool ret = true;
    amtec_mutex_.lock();
    amtecSetTargetVel(&amtec_->dev, amtec_->pan.id, req.velocity_pan);
    amtecSetTargetVel(&amtec_->dev, amtec_->tilt.id, req.velocity_tilt);
    amtec_mutex_.unlock();
    return ret;
  }

  bool sweepPan(amtec::SweepPan::Request& req, amtec::SweepPan::Response& resp)
  {
    bool ret;
    amtec_mutex_.lock();
    ret = amtecMotionFCosLoop(&amtec_->dev, amtec_->pan.id, req.sweep_amplitude, req.sweep_period);
    amtec_mutex_.unlock();
    return ret;
  }

  bool sweepTilt(amtec::SweepTilt::Request& req, amtec::SweepTilt::Response& resp)
  {
    bool ret;
    amtec_mutex_.lock();
    ret = amtecMotionFCosLoop(&amtec_->dev, amtec_->tilt.id, req.sweep_amplitude, req.sweep_period);
    amtec_mutex_.unlock();
    return ret;
  }

  bool spin()
  {
    ros::Rate r(100); // 10 ms or 100 Hz
    while (node_.ok())
    {
      amtec_mutex_.lock();
      amtec_pan_state_message_.state = amtecGetCubeState(&amtec_->dev, amtec_->pan.id);
      amtec_pan_state_message_.position = -amtecGetActPos(&amtec_->dev, amtec_->pan.id);
      amtec_pan_state_message_.velocity = -amtecGetActVel(&amtec_->dev, amtec_->pan.id);
      amtec_tilt_state_message_.state = amtecGetCubeState(&amtec_->dev, amtec_->tilt.id);
      amtec_tilt_state_message_.position = amtecGetActPos(&amtec_->dev, amtec_->tilt.id);
      amtec_tilt_state_message_.velocity = amtecGetActVel(&amtec_->dev, amtec_->tilt.id);
      //TODO(duhadway): check assumption that positions are given in radians
      parent_to_amtec_.setRotation(btQuaternion( amtec_pan_state_message_.position, -amtec_tilt_state_message_.position, 0 ));
      amtec_mutex_.unlock();

      pan_state_pub_.publish(amtec_pan_state_message_);
      tilt_state_pub_.publish(amtec_tilt_state_message_);
      tf_.sendTransform( tf::StampedTransform(parent_to_amtec_, ros::Time::now(), parent_frame_, amtec_frame_));

      ros::spinOnce();
      r.sleep();
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amtec");
  AmtecNode a;
  a.spin();
  return 0;
}
