/* 
This software is meant to localize an ARDrone
*/

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ardrone_brown/Navdata.h>
#include <ardrone_lclzr/drone_transform.h>

#include "ardrone_state.h"
#include <ar_recog/Tags.h>
#include <ar_recog/Tag.h>
#include "tagserver.h"
#include "ardrone_pf.h"
#include "ardrone_lclzr/ardrone_mean_state.h"
#include "quat_utils.h"

#ifndef PI
#define PI 3.1415926535
#endif

//#define _IMU_VX_MULT_ 0.54
//#define _IMU_VY_MULT_ 0.54

#define _IMU_VX_MULT_ 0.6
#define _IMU_VY_MULT_ 0.6

#define _ARDRONE_STARTX_ 8.21
#define _ARDRONE_STARTY_ 41.885

//#define _ARDRONE_STARTX_ 6.85
//#define _ARDRONE_STARTY_ 43.05

//#define __DEBUG_LEVEL_1
#define __DEBUG_LEVEL_2

double avg_posx;
double avg_posy;

ros::Publisher dronetransform_pub;
//tf::TransformBroadcaster *br;
ros::Publisher particles_visualization;
ros::Publisher avgpos_visualization;
ros::Publisher artag_visualization;
ros::Publisher pub_mean_state;
ardrone_state drone_state;
ardrone_particle_filter ardrone_pfilter;
vector<tagPose> tagposes;

void getSkewSymMatrix(const btVector3& ang_vel, btMatrix3x3& ssmat)
{
  ssmat[0][0]=ssmat[1][1]=ssmat[2][2]=0.0;
  ssmat[0][1]=-ang_vel[2];
  ssmat[1][0]=ang_vel[2];
  ssmat[0][2]=ang_vel[1];
  ssmat[2][0]=-ang_vel[1];
  ssmat[1][2]=-ang_vel[0];
  ssmat[2][1]=ang_vel[0];
}

// This gives the XYZ Euler angle rotation matrix as per Craig's book pg 442 Appendix B
// phi is the roll about the body fixed X axis
// theta is the pitch about the body fixed Y axis
// psi is the yaw about the body fixed Z axis
// Input angles are in radians
void getBodyFixedEulerRPYRotationMatrix(double rphi, double rth, double rpsi, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cphi = cos(rphi);
  double cth = cos(rth);
  double cpsi = cos(rpsi);
  double sphi = sin(rphi);
  double sth = sin(rth);
  double spsi = sin(rpsi);
  double sphisth = sphi*sth;
  double cphispsi = cphi*spsi;
  double cphicpsi = cphi*cpsi;

  xx = cth*cpsi;
  xy = -cth*spsi;
  xz = sth;
  yx = sphisth*cpsi+cphispsi;
  yy = -sphisth*spsi+cphicpsi;
  yz = -sphi*cth;
  zx = -cphicpsi*sth+sphi*spsi;
  zy = cphispsi*sth+sphi*cpsi;
  zz = cphi*cth;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// This returns a yaw(z axis) rotation by rpsi radians
void getYawRotationMatrix(double rpsi, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cpsi = cos(rpsi);
  double spsi = sin(rpsi);

  xx = cpsi;
  xy = -spsi;
  xz = 0.0;
  yx = spsi;
  yy = cpsi;
  yz = 0.0;
  zx = 0.0;
  zy = 0.0;
  zz = 1.0;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// This returns a Pitch(Y axis) rotation by rth radians
void getPitchRotationMatrix(double rth, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cth = cos(rth);
  double sth = sin(rth);

  xx = cth;
  xy = 0.0;
  xz = sth;
  yx = 0.0;
  yy = 1.0;
  yz = 0.0;
  zx = -sth;
  zy = 0.0;
  zz = cth;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// This returns a Roll(X axis) rotation by rphi radians
void getRollRotationMatrix(double rphi, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cphi = cos(rphi);
  double sphi = sin(rphi);

  xx = 1.0;
  xy = 0.0;
  xz = 0.0;
  yx = 0.0;
  yy = cphi;
  yz = -sphi;
  zx = 0.0;
  zy = sphi;
  zz = cphi;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

void getYawPitchRollRotationMatrix(double rpsi, double rth, double rphi, btMatrix3x3 &retMat)
{
  btMatrix3x3 rpsiRot, rthRot, rphiRot;
  getYawRotationMatrix(rpsi, rpsiRot);
  getPitchRotationMatrix(rth, rthRot);
  getRollRotationMatrix(rphi, rphiRot);
  retMat = (rpsiRot*(rthRot*rphiRot));
}

void publish_particles()
{
  visualization_msgs::MarkerArray marray;
  vector<ardrone_particle>::iterator pf_it = ardrone_pfilter.vec_pf->begin();
  int i=0;
  btVector3 rotaxis(0,0,1);

  btVector3 avg_pos(0,0,0);
  btVector3 avg_rpy(0,0,0);

  ardrone_lclzr::ardrone_mean_state mean_state;

  //this is used to convert from [0,2*PI] or not.
  double yaw_correction=0.0;

  if((ardrone_pfilter.yaw_min<(PI/2.0))&&(ardrone_pfilter.yaw_max>(3*PI/2.0)))
    {
      yaw_correction=(-2*PI);
    }

  for(;pf_it<ardrone_pfilter.vec_pf->end();pf_it++)
    {
      visualization_msgs::Marker tm;
      tm.header.frame_id = "/map";
      tm.header.stamp = ros::Time();
      tm.ns = "ardrone_particle";
      tm.id = i;
      tm.type = visualization_msgs::Marker::ARROW;
      tm.action = visualization_msgs::Marker::ADD;
      tm.pose.position.x = (*pf_it).w_pos[0];
      tm.pose.position.y = (*pf_it).w_pos[1];
      tm.pose.position.z = 1;
      btQuaternion qrot(rotaxis,((*pf_it).drone_rpy[2]));
      tm.pose.orientation.x = qrot.x();
      tm.pose.orientation.y = qrot.y();
      tm.pose.orientation.z = qrot.z();
      tm.pose.orientation.w = qrot.w();
      tm.scale.x = 1.20;
      tm.scale.y = 1.20;
      tm.scale.z = 1.20;
      tm.color.a = 0.6;
      tm.color.r = 0.0;
      tm.color.g = 1.0;
      tm.color.b = 0.0;
      marray.markers.push_back(tm);
      avg_pos[0]+=(*pf_it).w_pos[0];
      avg_pos[1]+=(*pf_it).w_pos[1];
      avg_rpy[2]+=(*pf_it).drone_rpy[2];
      
      if((*pf_it).drone_rpy[2]>(3*PI/2.0))
	avg_rpy[2]+=yaw_correction;
	
      i++;
    }

  //note: avg_rpy[2] is from [0,2*PI]
  avg_pos[0]=(avg_pos[0]/i);
  avg_pos[1]=(avg_pos[1]/i);
  avg_rpy[2]=(avg_rpy[2]/i);

  if(avg_rpy[2]<0)
    avg_rpy[2]+=(2*PI);

  /*
  //convert avg_rpy[2] from [0,2*PI] to [-PI,PI]
  if(avg_rpy[2]>PI)
    {
      avg_rpy[2]=(avg_rpy[2]-(2*PI));
    }
  */


  visualization_msgs::Marker avgpos;
  avgpos.header.frame_id = "/map";
  avgpos.header.stamp = ros::Time();
  avgpos.ns = "ardrone_avgpos";
  avgpos.id = 0;
  avgpos.type = visualization_msgs::Marker::ARROW;
  avgpos.action = visualization_msgs::Marker::ADD;
  avgpos.pose.position.x = avg_pos[0];
  avgpos.pose.position.y = avg_pos[1];
  avgpos.pose.position.z = 1;
  btQuaternion qrottemp(rotaxis,avg_rpy[2]);
  avgpos.pose.orientation.x = qrottemp.x();
  avgpos.pose.orientation.y = qrottemp.y();
  avgpos.pose.orientation.z = qrottemp.z();
  avgpos.pose.orientation.w = qrottemp.w();
  avgpos.scale.x = 1.8;
  avgpos.scale.y = 1.8;
  avgpos.scale.z = 1.8;
  avgpos.color.a = 1.0;
  avgpos.color.r = 1.0;
  avgpos.color.g = 0.0;
  avgpos.color.b = 0.0;

  double avg_rpy_deg = (avg_rpy[2]/PI*180);
  cout << "publish_particles: \n";
  cout << "avg_pos[0]: " << avg_pos[0] << "\n";
  cout << "avg_pos[1]: " << avg_pos[1] << "\n";
  cout << "avg_rpy[2]: " << avg_rpy[2] << "\n";
  cout << "avg_rpy_deg[2]: " << avg_rpy_deg << "\n\n";

  avg_posx = avg_pos[0];
  avg_posy = avg_pos[1];

  mean_state.x = avg_pos[0];
  mean_state.y = avg_pos[1];
  mean_state.thz = avg_rpy[2];
  mean_state.vx_droneframe = drone_state.b_lin_vel[0];
  mean_state.vy_droneframe = drone_state.b_lin_vel[1];
  mean_state.dt = drone_state.dt;

  pub_mean_state.publish(mean_state);
  particles_visualization.publish(marray);
  avgpos_visualization.publish(avgpos);
}

#ifdef __DEBUG_LEVEL_2
btVector3 w_drone_vel;
btVector3 w_drone_pos;
#endif

void navdataCallback(const ardrone_brown::Navdata::ConstPtr& navmsg)
{
  std_msgs::String msg;
  std::stringstream ss;

  if (drone_state.seqcnt==0)
    {
      drone_state.seqcnt++;
      ss << "First update to drone state.\n";
      
      //converting orientation data to radians
      double psi = navmsg->rotZ;
      double rpsi = psi/180.0*PI;
      //change rpsi from [-PI,+PI] to [0,2*PI]
      if(rpsi<0)
	{
	  rpsi = (rpsi+2*PI);
	}

      drone_state.q_yaw.setRPY(0.0,0.0,0.0);
      drone_state.q_yaw_prev.setRPY(0.0,0.0,0.0);
      drone_state.q_yaw_init.setRPY(0.0,0.0,rpsi);

      drone_state.rpsi=0.0;
      drone_state.rpsi_prev=0.0;
      drone_state.rpsi_init=rpsi;

      drone_state.tm=navmsg->tm;
      drone_state.dt=0.0;

      for(int i=0; i<3; i++)
	drone_state.b_lin_vel[i]=0.0;

      msg.data=ss.str();
      ROS_INFO("%s", msg.data.c_str());
    }
  else
    {
      double dt = navmsg->tm - drone_state.tm;
      if(dt<0)
	{
	  // navdata message received out of sync, so ignore it
	  /*
	  ss << "\n\nnavdata message received with out of sync time stamp.\n";
	  ss << "dt(usec): " << dt << "\n\n";
	  msg.data=ss.str();
	  ROS_INFO("%s", msg.data.c_str());
	  */
	}
      else
	{
	  drone_state.seqcnt++;
	  
	  //convert dt from microseconds to seconds
	  dt=dt/1000000.0;
	  drone_state.dt=dt;
	  drone_state.tm=navmsg->tm;

	  double psi = navmsg->rotZ;
	  double rpsi = psi/180.0*PI;
	  //change rpsi from [-PI,+PI] to [0,2*PI]
	  if(rpsi<0)
	    {
	      rpsi = (rpsi+2*PI);
	    }

#ifdef __DEBUG_LEVEL_2
	  btMatrix3x3 w_drone_rotation;
	  getYawRotationMatrix(rpsi,w_drone_rotation);
	  btVector3 tdrone_vel(((_IMU_VX_MULT_*navmsg->vx)/1000.0),((_IMU_VX_MULT_*navmsg->vy)/1000.0),(navmsg->vz/1000.0));
	  btVector3 tw_vel = (w_drone_rotation*tdrone_vel);
	  btVector3 tw_avg_vel = ((tw_vel+w_drone_vel)/2.0);
	  btVector3 tw_dr_disp = (dt*tw_avg_vel);
	  w_drone_pos = (w_drone_pos+tw_dr_disp);
	  w_drone_vel = tw_vel;
	  cout << "w_drone_pos: (" << w_drone_pos[0] << "," << w_drone_pos[1] 
	       << "," << w_drone_pos[2] << ")\n";
#endif

	  drone_state.q_yaw_prev = drone_state.q_yaw;
	  drone_state.q_yaw.setRPY(0.0,0.0,rpsi);

#ifdef __DEBUG_LEVEL_2
	  cout << "psi(deg): " << (rpsi/PI*180.0) << "\n";
	  cout << "1.q_yaw.getAngle(): " << ((drone_state.q_yaw.getAngle())/PI*180.0) << "\n";
#endif

	  drone_state.q_yaw = getQuaternionDiff(drone_state.q_yaw, drone_state.q_yaw_init);

#ifdef __DEBUG_LEVEL_2
	  cout << "2.q_yaw.getAngle(): " << ((drone_state.q_yaw.getAngle())/PI*180.0) << "\n";
	  cout << "q_yaw_init.getAngle(): " << ((drone_state.q_yaw_init.getAngle())/PI*180.0) << "\n";
#endif

	  drone_state.b_lin_vel[0]=((_IMU_VX_MULT_*navmsg->vx)/1000.0);
	  drone_state.b_lin_vel[1]=((_IMU_VX_MULT_*navmsg->vy)/1000.0);
	  drone_state.b_lin_vel[2]=(navmsg->vz/1000.0);

#ifdef __DEBUG_LEVEL_1
	  ss << "rpsi: " << rpsi << "\n";

	  ss << "drone_state.q_yaw(rad): " 
	     << getQuaternionAngleRad(drone_state.q_yaw) << "\n";
	  ss << "drone_state.q_yaw(deg): " 
	     << getQuaternionAngleDeg(drone_state.q_yaw) << "\n";

	  ss << "drone_state.q_yaw_prev(rad): " 
	     << getQuaternionAngleRad(drone_state.q_yaw_prev) << "\n";
	  ss << "drone_state.q_yaw_prev(deg): " 
	     << getQuaternionAngleDeg(drone_state.q_yaw_prev) << "\n";

	  ss << "drone_state.q_yaw_init(rad): " 
	     << getQuaternionAngleRad(drone_state.q_yaw_init) << "\n";
	  ss << "drone_state.q_yaw_init(deg): " 
	     << getQuaternionAngleDeg(drone_state.q_yaw_init) << "\n";

	  ss << "\n";
#endif

	  ardrone_pfilter.perform_motion_update(drone_state);

	  publish_particles();

#ifdef __DEBUG_LEVEL_1
	  msg.data=ss.str();
	  ROS_INFO("%s", msg.data.c_str());
#endif

	}
    }

}

void tagscallback(const ar_recog::Tags::ConstPtr& tagsmsg)
{
  ardrone_pfilter.perform_measurement_update(tagsmsg,tagposes,avg_posx,avg_posy,artag_visualization);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ardrone_lclzr");
  /*
  drone_state.state_initialize(btVector3(0.0,0.0,0.0),btVector3(0.0,0.0,0.0), \
			       btVector3(0.0,0.0,0.0),btVector3(0.0,0.0,0.0),\
			       btVector3(0.0,0.0,0.0),btVector3(0.0,0.0,0.0),\
			       0.0,0);
  */
  //br = new tf::TransformBroadcaster();
  //drone_state.initialize_pose(btVector3(8.21,41.885,0.0),btVector3(0.0,0.0,0.0));
  //drone_state.initialize_pose(btVector3(6.85,43.05,0.0),btVector3(0.0,0.0,0.0));
  drone_state.initialize_pose(btVector3(_ARDRONE_STARTX_,_ARDRONE_STARTY_,0.0),btVector3(0.0,0.0,0.0));

  //ardrone_pfilter.initialize_particles(btVector3(8.21,41.885,0.0),btVector3(0.0,0.0,0.0));
  //ardrone_pfilter.initialize_particles(btVector3(6.85,43.05,0.0),btVector3(0.0,0.0,0.0));
  ardrone_pfilter.initialize_particles(btVector3(_ARDRONE_STARTX_,_ARDRONE_STARTY_,0.0),btVector3(0.0,0.0,0.0));

  avg_posx=_ARDRONE_STARTX_;
  avg_posy=_ARDRONE_STARTY_;

  ros::NodeHandle n;

#ifdef __DEBUG_LEVEL_2
  w_drone_vel=btVector3(0.0,0.0,0.0);
  //w_drone_pos=btVector3(8.21,41.885,0.0);
  //w_drone_pos=btVector3(6.85,43.05,0.0);
  w_drone_pos=btVector3(_ARDRONE_STARTX_,_ARDRONE_STARTY_,0.0);
#endif

  //initialize tagposes

  /*
  tagPose tag1=tagPose(0,5.85,43.05,-PI/2);
  tagposes.push_back(tag1);

  //tagPose tag2=tagPose(1,9.21,42.65,-PI/2);
  tagPose tag2=tagPose(1,9.21,41.885,-PI/2);
  tagposes.push_back(tag2);
  */





  //tagPose tag1=tagPose(0,9.21,41.885,-PI/2);
  tagposes.push_back(tagPose(0,9.21,41.885,-PI/2));
  tagposes.push_back(tagPose(0,25.2064,34.461,-PI/2));

  tagposes.push_back(tagPose(1,13.33,41.8,-PI/2));
  tagposes.push_back(tagPose(1,25.2064,32.461,-PI/2));

  tagposes.push_back(tagPose(3,17.8514,41.773,-PI/2));
  tagposes.push_back(tagPose(3,25.2064,30.461,-PI/2));

  tagposes.push_back(tagPose(4,20.5163,42.0439,-PI/2));
  tagposes.push_back(tagPose(4,25.2064,28.461,-PI/2));

  tagposes.push_back(tagPose(5,23.1163,42.0439,-PI/2));
  tagposes.push_back(tagPose(5,24.8924,27.366,-PI/2));

  tagposes.push_back(tagPose(6,24.1163,42.0439,-PI/2));
  tagposes.push_back(tagPose(6,24.5795,25.6474,-PI/2));  

  tagposes.push_back(tagPose(8,25.2063,41.6039,-PI/2));
  tagposes.push_back(tagPose(8,23.6941,24.2592,-PI/2));

  tagposes.push_back(tagPose(10,25.2064,40.2805,-PI/2));
  tagposes.push_back(tagPose(10,21.1235,21.2592,-PI/2));

  tagposes.push_back(tagPose(12,25.2064,37.6105,-PI/2));
  tagposes.push_back(tagPose(12,18.8635,18.9992,-PI/2));




  /*
  tagposes.push_back(tagPose(12,25.4464,38.3805,-PI/2));
  tagposes.push_back(tagPose(12,18.8635,18.9992,-PI/2));
  */

  /*
  tagPose tag6=tagPose(5,25.1512,41.6566,-PI/2);
  tagposes.push_back(tag6);

  tagPose tag7=tagPose(6,25.4578,40.5917,-PI/2);
  tagposes.push_back(tag7);

  tagPose tag8=tagPose(8,25.5224,36.461,-PI/2);
  tagposes.push_back(tag8);

  tagPose tag9=tagPose(10,25.5708,31.1362,-PI/2);
  tagposes.push_back(tag9);

  tagPose tag10=tagPose(11,25.5103,28.5868,-PI/2);
  tagposes.push_back(tag10);

  tagPose tag11=tagPose(12,24.7595,25.6474,-PI/2);
  tagposes.push_back(tag11);
  */

  /*
  tagPose tag2=tagPose(0,9.21,41.885,-PI/2);
  tagposes.push_back(tag2);

  tagPose tag3=tagPose(1,13.33,41.8,-PI/2);
  tagposes.push_back(tag3);

  tagPose tag4=tagPose(3,17.8514,41.773,-PI/2);
  tagposes.push_back(tag4);

  tagPose tag5=tagPose(4,20.5163,42.0439,-PI/2);
  tagposes.push_back(tag5);

  tagPose tag6=tagPose(5,25.1512,41.6566,-PI/2);
  tagposes.push_back(tag6);

  tagPose tag7=tagPose(6,25.4578,40.5917,-PI/2);
  tagposes.push_back(tag7);

  tagPose tag8=tagPose(8,25.5224,36.461,-PI/2);
  tagposes.push_back(tag8);

  tagPose tag9=tagPose(10,25.5708,31.1362,-PI/2);
  tagposes.push_back(tag9);

  tagPose tag10=tagPose(11,25.5103,28.5868,-PI/2);
  tagposes.push_back(tag10);

  tagPose tag11=tagPose(12,24.7595,25.6474,-PI/2);
  tagposes.push_back(tag11);
  */

  /*
  tagPose tag12=tagPose(11,23.6941,24.2592,-PI/2);
  tagposes.push_back(tag12);

  tagPose tag13=tagPose(12,21.1235,21.2592,-PI/2);
  tagposes.push_back(tag13);

  tagPose tag14=tagPose(13,18.8635,18.9992,-PI/2);
  tagposes.push_back(tag14);
  */

  ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1, &navdataCallback);
  //ros::Publisher dronestate_pub = n.advertise<std_msgs::String>("/ardrone_lclzr/dronestate",100);

  ros::Subscriber tagsmsg_sub = n.subscribe("tags", 1, &tagscallback);

  //particles_pub = n.advertise<ardrone_lclzr::ardrone_pf_particles>("/ardrone/pf_particles",1);

  particles_visualization = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);

  avgpos_visualization = n.advertise<visualization_msgs::Marker>("visualization_marker",1);

  artag_visualization = n.advertise<visualization_msgs::Marker>("visualization_marker",1);

  pub_mean_state = n.advertise<ardrone_lclzr::ardrone_mean_state>("ardrone_mean_state",1);
  //dronetransform_pub = n.advertise<ardrone_lclzr::drone_transform>("/ardrone/drone_transform",100);

  ros::Rate loop_rate(60);

  //int count=0;
  while (ros::ok())
    {

      //dronestate_pub.publish(msg);
      ros::spinOnce();

      loop_rate.sleep();
      //++count;
    }

  return 0;
}


