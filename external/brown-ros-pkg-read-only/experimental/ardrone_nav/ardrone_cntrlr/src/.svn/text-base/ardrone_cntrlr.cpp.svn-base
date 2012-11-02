#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <time.h>

#include "ardrone_lclzr/ardrone_mean_state.h"
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#define PI 3.1415926535

#define __TRACKER_SKIP_CNT 2

#define _MAX_U_LIMIT_ 0.09
//#define _KP_ 0.039
//#define _KV_ 0.021
//#define _KP_ 0.045
//#define _KV_ 0.021


#define _KP_ 0.042
#define _KV_ 0.018
#define _KPTHZ_ 0.1
#define _KVTHZ_ 0.04
#define _WT_ 0.003

#define _WT_THZ_ 0.003

/*
#define _KP_ 0.036
#define _KV_ 0.015
#define _KPTHZ_ 0.09
#define _KVTHZ_ 0.03
*/

//#define _KPTHZ_ 0.0
//#define _KVTHZ_ 0.0
#define _KI_ 0.06
#define _KITHZ_ 0.06


//#define _WT_ 0.0

//#define _WT_ 0.0006
//#define _WT_THZ_ 0.009

#define __DEBUG_LEVEL_1

#define _ARDRONE_STARTX_ 8.21
#define _ARDRONE_STARTY_ 41.885

//#define _ARDRONE_STARTX_ 6.85
//#define _ARDRONE_STARTY_ 43.05

/*
#define __TRACKER_SKIP_CNT 3
#define _KP_ 0.08
#define _KV_ 0.0565
#define _KPTHZ_ 0.012
#define _KVTHZ_ 0.00693
#define _KI_ 0.06
#define _KITHZ_ 0.06
#define _WT_ 0.001
#define __DEBUG_LEVEL_1
*/

using namespace std;

deque<geometry_msgs::PoseStamped> planner_path;

double ardrone_lclzrx;
double ardrone_lclzry;
double ardrone_lclzrvx_droneframe;
double ardrone_lclzrvy_droneframe;
double ardrone_lclzrthz;

//PID controller variables
double xdes;
double ydes;
double xdes_p;
double ydes_p;
double xdes_dot;
double ydes_dot;
double xdes_dot_p;
double ydes_dot_p;
double xdes_ddot;
double ydes_ddot;
double thz;
double thz_p;
double thz_dot;
double thz_dot_p;
double vxdrone_p;
double vydrone_p;

double xerr;
double yerr;
double xerr_p;
double yerr_p;
double xerr_i;
double yerr_i;
double thzerr;
double thzerr_p;
double thzdes;
double thzdes_p;
double thzdes_dot;
double thzdes_dot_p;

double kp;
double kv;
double kpthz;
double kvthz;
double ki;
double kithz;
double des_traj_wt;
double des_thz_wt;
double tr_ux;
double tr_uy;
double tr_uthz;

bool traj_track_mode;

unsigned int cntr;

double drone_time;
double drone_time_last;

ros::Publisher cmdvel_pub;

int track_trajectory();

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

void append_onto_path(double xg, double yg)
{

  //dividing dist into 5cm steps
  double xd=(xg-ardrone_lclzrx);
  double yd=(yg-ardrone_lclzry);
  double dg=sqrt(xd*xd+yd*yd);
  unsigned int dsteps=(unsigned int)((dg/0.05)+0.5);
  double xinc=(xd/(dsteps*1.0));
  double yinc=(yd/(dsteps*1.0));

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/map";
  pose.pose.position.x = ardrone_lclzrx;
  pose.pose.position.y = ardrone_lclzry;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  planner_path.push_back(pose);
  
  for(unsigned int i=0; i<dsteps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "/map";
      pose.pose.position.x = (ardrone_lclzrx+xinc);
      pose.pose.position.y = (ardrone_lclzry+yinc);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      planner_path.push_back(pose);
    }

}

void path_callback(const nav_msgs::PathConstPtr& path)
{

  unsigned int path_size = planner_path.size();

  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	planner_path.pop_front();
      planner_path.clear();
      cout << "ardrone_cntrlr. planner path cleared.\n";
    }

  unsigned int gpath_size = path->poses.size();
  
  if(gpath_size>0)
    {
      for(unsigned int i=0; i<1; i++)
	{
	  geometry_msgs::PoseStamped pose;
	  pose.header.stamp = ros::Time::now();
	  pose.header.frame_id = "/map";
	  pose.pose.position.x = ardrone_lclzrx;
	  pose.pose.position.y = ardrone_lclzry;
	  pose.pose.position.z = 0.0;
	  pose.pose.orientation.x = 0.0;
	  pose.pose.orientation.y = 0.0;
	  pose.pose.orientation.z = 0.0;
	  pose.pose.orientation.w = 1.0;
	  planner_path.push_back(pose);
	}
      
      for(unsigned int i=0; i<gpath_size; i++)
	{
	  geometry_msgs::PoseStamped pose;
	  pose = path->poses[i];
	  planner_path.push_back(pose);
	}
      cout << "path received. planner_path.size(): " << planner_path.size() << "\n";
      
      drone_time=0.0;
      drone_time_last=0.0;
      
      xdes=-9999;
      ydes=-9999;
      xdes_p=-9999;
      ydes_p=-9999;
      xdes_dot=-9999;
      ydes_dot=-9999;
      xdes_dot_p=-9999;
      ydes_dot_p=-9999;
      xdes_ddot=-9999;
      ydes_ddot=-9999;
      thz=-9999;
      thz_p=-9999;
      thz_dot=-9999;
      thz_dot_p=-9999;
      vxdrone_p=-9999;
      vydrone_p=-9999;
      
      xerr=-9999;
      yerr=-9999;
      xerr_p=-9999;
      yerr_p=-9999;
      xerr_i=-9999;
      yerr_i=-9999;
      thzerr=-9999;
      thzerr_p=-9999;
      thzdes=-9999;
      thzdes_p=-9999;
      thzdes_dot=-9999;
      thzdes_dot_p=-9999;

      tr_ux=1.0;
      tr_uy=1.0;
      tr_uthz=1.0;
      
      //send a zero twist command to the drone
      geometry_msgs::Twist cmdtwist;
      cmdtwist.linear.x=0.0;
      cmdtwist.linear.y=0.0;
      cmdtwist.linear.z=0.0;
      cmdtwist.angular.x=0.0;
      cmdtwist.angular.y=0.0;
      cmdtwist.angular.z=0.0;
      cmdvel_pub.publish(cmdtwist);
      
    }
  else
    {
      cout << "path_callback. error. planner_path.size(): " << gpath_size << "\n";
    }

}

void lclzr_callback(const ardrone_lclzr::ardrone_mean_state& drone_pose)
{
  ardrone_lclzrx=drone_pose.x;
  ardrone_lclzry=drone_pose.y;
  ardrone_lclzrthz=drone_pose.thz;
  ardrone_lclzrvx_droneframe=drone_pose.vx_droneframe;
  ardrone_lclzrvy_droneframe=drone_pose.vy_droneframe;
  drone_time+=drone_pose.dt;
  if(traj_track_mode)
    track_trajectory();
}

int track_trajectory()
{

  if(planner_path.size()==0)
    {
      cout << "trajectory is empty.\n\n";
      return 0;
    }

  cntr++;

  if((cntr%__TRACKER_SKIP_CNT)!=0)
    {
      if(planner_path.size()>1)
	planner_path.pop_front();
      return 1;
    }

  cntr=0;
  
  double drone_dt=(drone_time-drone_time_last);
  drone_time_last=drone_time;

  thz_p=thz;
  thz=ardrone_lclzrthz;

  /*
  thz_dot_p=thz_dot;

  if((drone_dt>0.01)&&(thz_p!=-9999))
    {
      //calculate ardrone angular velocity about its z axis
      double thzdiff=0.0;
      if(fabs(thz)<0.001) thz=0.0;
      if(fabs(thz_p)<0.001) thz_p=0.0;
      thzdiff = (thz-thz_p);      
      if(thzdiff<(-PI))
	thzdiff=(thzdiff+2*PI);
      else if(thzdiff>PI)
	thzdiff=(thzdiff-2*PI);
      thz_dot=(thzdiff/drone_dt);
    }

  double thz_ddot=0.0;

  if((drone_dt>0.01)&&(thz_dot_p!=-9999))
    {
      thz_ddot = ((thz_dot-thz_dot_p)/drone_dt);
    }
  */

  if(planner_path.size()>1)
    planner_path.pop_front();
  
  geometry_msgs::PoseStamped pose;
  pose=planner_path.front();

  xdes_p = xdes;
  ydes_p = ydes;

  xdes = pose.pose.position.x;
  ydes = pose.pose.position.y;

  xerr_p = xerr;
  yerr_p = yerr;

  double xerr_map = (xdes-ardrone_lclzrx);
  double yerr_map = (ydes-ardrone_lclzry);

  thzerr_p = thzerr;

  /*
  if((xdes!=xdes_p)||(ydes!=ydes_p))
    {
      double xdes_diff = (xdes-xdes_p);
      double ydes_diff = (ydes-ydes_p);
      thzdes = atan2(ydes_diff,xdes_diff);
      //change thzdes to {[+PI,-PI]}
      if(thzdes>PI)
	{
	  thzdes = (thzdes-2*PI);
	}
      else if(thzdes<-PI)
	{
	  thzdes = (2*PI+thzdes);
	}
	
    }
  */

  bool setdefault_thz_des=false;

  thzdes_p = thzdes;

  if(planner_path.size()==1)
    {

      tr_ux=1.0;
      tr_uy=1.0;
      tr_uthz=1.0;

      /*
      tr_ux=1.0;
      tr_uy=1.0;
      tr_uthz=0.1;
      */
    }
  else if(planner_path.size()>1)
    {
      if(xdes_p!=-9999)
	{
	  double xdes_diff = (xdes-xdes_p);
	  double ydes_diff = (ydes-ydes_p);
	  thzdes = atan2(ydes_diff,xdes_diff);
	  /*
	  //change thzdes to {[+PI,-PI]}
	  if(thzdes>PI)
	    {
	      thzdes = (thzdes-2*PI);
	    }
	  else if(thzdes<-PI)
	    {
	      thzdes = (2*PI+thzdes);
	    }
	  */

	  //change thzdes to [0,2*PI]
	  if(thzdes<0)
	    thzdes = (thzdes+2*PI);

	  if(fabs(thzdes)>(2*PI))
	    {
	      cout << "track_trajectory. bad thzdes value.\n";
	      thzdes = ardrone_lclzrthz;
	      setdefault_thz_des=true;
	    }
	}
      else
	{
	  setdefault_thz_des=true;
	  thzdes = ardrone_lclzrthz;
	}
    }

  thzerr=0.0;
  if((thzdes!=-9999)&&(!setdefault_thz_des))
    {
      if(fabs(thzdes)<0.001) thzdes=0.0;
      if(fabs(thz)<0.001) thz=0.0;
      
      thzerr = (thzdes-thz);
      if((thzerr<(-PI))&&((thzdes<=(PI/2.0))&&(thzdes>=0.0))&&((thz>=(3*PI/2.0))&&(thz<=(2*PI))))
	{
	  thzerr = (thzerr+2*PI);
	}
      else if((thzerr>PI)&&((thz<=(PI/2.0))&&(thz>=0.0))&&((thzdes>=(3*PI/2.0))&&(thzdes<=(2*PI))))
	{
	  thzerr = (thzerr-2*PI);
	}
    }

  double thzerr_dot=0.0;

  if((drone_dt>0.01)&&(thzerr_p!=-9999))
    {
      double thzdiff=0.0;
      thzdiff = (thzerr-thzerr_p);
      thzerr_dot = (thzdiff/drone_dt);
      if(setdefault_thz_des)
	thzerr_dot=0.0;
    }

  thzdes_dot_p = thzdes_dot;

  //calculate thzdes_dot
  if((drone_dt>0.01)&&(thzdes_p!=-9999))  
    {
      double thzdiff=0.0;
      if(fabs(thzdes)<0.001) thzdes=0.0;
      if(fabs(thzdes_p)<0.001) thzdes_p=0.0;
      thzdiff = (thzdes-thzdes_p);      

      if((thzdiff<(-PI))&&((thzdes<=(PI/2.0))&&(thzdes>=0.0))&&((thzdes_p>=(3*PI/2.0))&&(thzdes_p<=(2*PI))))
	{
	  thzdiff = (thzdiff+2*PI);
	}
      else if((thzdiff>PI)&&((thzdes_p<=(PI/2.0))&&(thzdes_p>=0.0))&&((thzdes>=(3*PI/2.0))&&(thzdes<=(2*PI))))
	{
	  thzdiff = (thzdiff-2*PI);
	}

      /*
      if(thzdiff<(-PI))
	thzdiff=(thzdiff+2*PI);
      else if(thzdiff>PI)
	thzdiff=(thzdiff-2*PI);
      */

      thzdes_dot=(thzdiff/drone_dt);
    }

  double thzdes_ddot=0.0;

  if((drone_dt>0.01)&&(thzdes_dot_p!=-9999))
    {
      thzdes_ddot = ((thzdes_dot-thzdes_dot_p)/drone_dt);
    }

  double cthz = cos(thz);
  double sthz = sin(thz);

  xerr = (cthz*xerr_map+sthz*yerr_map);
  yerr = (-sthz*xerr_map+cthz*yerr_map);

  double xerr_dot=0.0;
  double yerr_dot=0.0;

  if((drone_dt>0.01)&&(xerr_p!=-9999))
    {
      xerr_dot = ((xerr-xerr_p)/drone_dt);
      yerr_dot = ((yerr-yerr_p)/drone_dt);
    }

  xdes_dot_p = xdes_dot;
  ydes_dot_p = ydes_dot;

  if((drone_dt>0.01)&&(xdes_p!=-9999))
    {
      xdes_dot = ((xdes-xdes_p)/drone_dt);
      ydes_dot = ((ydes-ydes_p)/drone_dt);
    }

  if((drone_dt>0.01)&&(xdes_dot_p!=-9999))
    {
      xdes_ddot = ((xdes_dot-xdes_dot_p)/drone_dt);
      ydes_ddot = ((ydes_dot-ydes_dot_p)/drone_dt);
    }

  double xdes_d_map=0.0;
  double ydes_d_map=0.0;
  double xdes_dd_map=0.0;
  double ydes_dd_map=0.0;

  /*
  double thd=0.0;
  double thdd=0.0;
  */

  if(xdes_dot!=-9999)
    {
      xdes_d_map=xdes_dot;
      ydes_d_map=ydes_dot;
    }

  if(xdes_ddot!=-9999)
    {
      xdes_dd_map=xdes_ddot;
      ydes_dd_map=ydes_ddot;
    }

  /*
  if(thz_dot!=-9999)
    thd=thz_dot;

  thdd=thz_ddot;
  */

  double dr_x_des = (cthz*xdes+sthz*ydes);
  double dr_y_des = (-sthz*xdes+cthz*ydes);

  double dr_vx_des = (cthz*xdes_d_map+sthz*ydes_d_map);
  double dr_vy_des = (-sthz*xdes_d_map+cthz*ydes_d_map);

  double dr_ax_des = (cthz*xdes_dd_map+sthz*ydes_dd_map);
  double dr_ay_des = (-sthz*xdes_dd_map+cthz*ydes_dd_map);

  /*
  //angular velocity of drone frame wrt map frame
  btVector3 m_w_d(0.0,0.0,thd);
  btMatrix3x3 m_w_d_ss;
  getSkewSymMatrix(m_w_d,m_w_d_ss);
  
  //angular acceleration of drone frame wrt map frame
  btVector3 m_a_d(0.0,0.0,thdd);
  btMatrix3x3 m_a_d_ss;
  getSkewSymMatrix(m_a_d,m_a_d_ss);

  //angular velocity of map frame wrt drone frame
  btVector3 d_w_m(0.0,0.0,-thd);
  btMatrix3x3 d_w_m_ss;
  getSkewSymMatrix(d_w_m,d_w_m_ss);
  
  //angular acceleration of map frame wrt drone frame
  btVector3 d_a_m(0.0,0.0,-thdd);
  btMatrix3x3 d_a_m_ss;
  getSkewSymMatrix(d_a_m,d_a_m_ss);
  */

  double vx_dr = ardrone_lclzrvx_droneframe;
  double vy_dr = ardrone_lclzrvy_droneframe;

  double axdrone_drone=0.0;
  double aydrone_drone=0.0;

  if((drone_dt>0.01)&&(vxdrone_p!=-9999))
    {
      axdrone_drone = ((vx_dr-vxdrone_p)/drone_dt);
      aydrone_drone = ((vy_dr-vydrone_p)/drone_dt);
    }

  vxdrone_p = vx_dr;
  vydrone_p = vy_dr;

  btVector3 dr_accdr(axdrone_drone,aydrone_drone,0.0);
  btVector3 dr_accdes(dr_ax_des,dr_ay_des,0.0);
  btVector3 dr_veldes(dr_vx_des,dr_vy_des,0.0);
  btVector3 dr_posdes(dr_x_des,dr_y_des,0.0);

  btVector3 dr_accdes_dr(0.0,0.0,0.0);
  //dr_accdes_dr = (dr_accdes - dr_accdr + (d_a_m_ss*dr_posdes) + (d_w_m_ss*(d_w_m_ss*dr_posdes)) + 2.0*(d_w_m_ss*dr_veldes));

  //dr_accdes_dr = dr_accdes - dr_accdr;
  dr_accdes_dr = dr_accdes;

  /*
  double ux = (drone_dt*(des_traj_wt*dr_accdes_dr[0] + kv*xerr_dot + kp*xerr));
  double uy = (drone_dt*(des_traj_wt*dr_accdes_dr[1] + kv*yerr_dot + kp*yerr));
  */

  double ux = (tr_ux*(des_traj_wt*dr_accdes_dr[0] + kv*xerr_dot + kp*xerr));
  double uy = (tr_uy*(des_traj_wt*dr_accdes_dr[1] + kv*yerr_dot + kp*yerr));
  //double uthz = (drone_dt*(des_thz_wt*(kv*thzerr_dot + kp*thzerr)));
  ///double uthz = (des_thz_wt*(kv*thzerr_dot + kp*thzerr));
  double uthz = (tr_uthz*(des_thz_wt*thzdes_ddot + kvthz*thzerr_dot + kpthz*thzerr));

#ifdef __DEBUG_LEVEL_1
  cout << "planner_path.size(): " << planner_path.size() << "\n";
  cout << "xdes,ydes: " << xdes << "," << ydes << "\n";
  cout << "lclzrx,lclzry: " << ardrone_lclzrx << "," << ardrone_lclzry << "\n";
  cout << "drone_dt: " << drone_dt << "\n";
  cout << "dr_accdes_dr[0],dr_accdes_dr[1]: "<<dr_accdes_dr[0]<<","<<dr_accdes_dr[1]<<"\n";
  cout << "xerr,yerr: " << xerr << "," << yerr << "\n";
  cout << "xerr_p,yerr_p: " << xerr_p << "," << yerr_p << "\n";
  cout << "xerr_dot,yerr_dot: " << xerr_dot << "," << yerr_dot << "\n";
  cout << "ux,uy: " << ux << "," << uy << "\n";
  cout << "lclzrthz, thzdes, thzdes_ddot: " << ardrone_lclzrthz << "," << thzdes 
       << "," << thzdes_ddot << "\n";
  cout << "thzerr, thzerr_p, thzerr_dot, uthz: " <<thzerr << "," << thzerr_p 
       << "," << thzerr_dot << "," << uthz << "\n\n";
#endif

  /*
  if(fabs(ux)>_MAX_U_LIMIT_) ux=(_MAX_U_LIMIT_*(ux/fabs(ux)));
  if(fabs(uy)>_MAX_U_LIMIT_) uy=(_MAX_U_LIMIT_*(uy/fabs(uy)));
  if(fabs(uthz)>_MAX_U_LIMIT_) uthz=(_MAX_U_LIMIT_*(uthz/fabs(uthz)));  
  */

  //command translation
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=ux;
  cmdtwist.linear.y=uy;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=uthz;
  cmdvel_pub.publish(cmdtwist);
  
  return 1;

}

void traj_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside trajectory tracking callback.\n\n";

  traj_track_mode=true;

  drone_time=0.0;
  drone_time_last=0.0;

  xdes=-9999;
  ydes=-9999;
  xdes_p=-9999;
  ydes_p=-9999;
  xdes_dot=-9999;
  ydes_dot=-9999;
  xdes_dot_p=-9999;
  ydes_dot_p=-9999;
  xdes_ddot=-9999;
  ydes_ddot=-9999;
  thz=-9999;
  thz_p=-9999;
  thz_dot=-9999;
  thz_dot_p=-9999;
  vxdrone_p=-9999;
  vydrone_p=-9999;

  xerr=-9999;
  yerr=-9999;
  xerr_p=-9999;
  yerr_p=-9999;
  xerr_i=-9999;
  yerr_i=-9999;
  thzerr=-9999;
  thzerr_p=-9999;
  thzdes=-9999;
  thzdes_p=-9999;
  thzdes_dot=-9999;
  thzdes_dot_p=-9999;

  tr_ux=1.0;
  tr_uy=1.0;
  tr_uthz=1.0;

  //send a zero twist command to the drone
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=0.0;
  cmdtwist.linear.y=0.0;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=0.0;
  cmdvel_pub.publish(cmdtwist);

}

void stop_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside stop_tracking_callback.\n";

  traj_track_mode=false;

  //send a zero twist command to the drone
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=0.0;
  cmdtwist.linear.y=0.0;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=0.0;
  cmdvel_pub.publish(cmdtwist);

  unsigned int path_size = planner_path.size();
  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	planner_path.pop_front();
      planner_path.clear();
      cout << "ardrone_cntrlr. planner path cleared.\n";
    }

}

int teleop_relay(const geometry_msgs::TwistConstPtr &msg)
{
  traj_track_mode=false;

#ifdef __DEBUG_LEVEL_2
  cout << "teleop_relay. cmd_vel msg: \n";
  cout << "msg->linear.(x,y,z): " << msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "\n";
  cout << "msg->angular.(x,y,z): " << msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "\n\n";
#endif
  
  geometry_msgs::Twist cmdtwist;
  cmdtwist=(*msg);
  cmdvel_pub.publish(cmdtwist);

  return 1;
}

void cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
{
  teleop_relay(msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ardrone_cntrlr");
  ros::NodeHandle n;

  cout << "Inside ardrone_cntrlr.\n";

  planner_path.clear();

  drone_time=0.0;
  drone_time_last=0.0;

  //Global variable initialization
  xdes=-9999;
  ydes=-9999;
  xdes_p=-9999;
  ydes_p=-9999;
  xdes_dot=-9999;
  ydes_dot=-9999;
  xdes_dot_p=-9999;
  ydes_dot_p=-9999;
  xdes_ddot=-9999;
  ydes_ddot=-9999;
  thz=-9999;
  thz_p=-9999;
  thz_dot=-9999;
  thz_dot_p=-9999;
  vxdrone_p=-9999;
  vydrone_p=-9999;

  xerr=-9999;
  yerr=-9999;
  xerr_p=-9999;
  yerr_p=-9999;
  xerr_i=-9999;
  yerr_i=-9999;
  thzerr=-9999;
  thzerr_p=-9999;
  thzdes=-9999;
  thzdes_p=-9999;
  thzdes_dot=-9999;
  thzdes_dot_p=-9999;

  kp=_KP_;
  //kv=(2.0*sqrt(kp));
  kv=_KV_;
  kpthz=_KPTHZ_;
  kvthz=_KVTHZ_;
  ki=_KI_;
  kithz=_KITHZ_;

  ///ardrone_lclzrx=6.85;
  ///ardrone_lclzry=43.05;

  ///ardrone_lclzrx=8.21;
  ///ardrone_lclzry=41.885;

  ardrone_lclzrx=_ARDRONE_STARTX_;
  ardrone_lclzry=_ARDRONE_STARTY_;

  ardrone_lclzrthz=0.0;
  ardrone_lclzrvx_droneframe=0.0;
  ardrone_lclzrvy_droneframe=0.0;

  traj_track_mode=false;

  des_traj_wt=_WT_;
  des_thz_wt=_WT_THZ_;
  tr_ux=1.0;
  tr_uy=1.0;
  tr_uthz=1.0;

  cntr=0;

  ros::Subscriber lclzr_sub = n.subscribe("/ardrone_mean_state",1,&lclzr_callback);
  ros::Subscriber plan_sub = n.subscribe("/plan",1,&path_callback);
  cmdvel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1);

  ros::Subscriber traj_tracking_sub = n.subscribe("/traj_tracking_start",1,&traj_tracking_callback);
  ros::Subscriber stop_tracking_sub = n.subscribe("/stop_tracking",1,&stop_tracking_callback);
  ros::Subscriber cmdvel_sub = n.subscribe("/ardrone_cntrlr/cmd_vel",1,&cmd_vel_callback);

  ros::Rate loop_rate(60);

  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 1;

}




