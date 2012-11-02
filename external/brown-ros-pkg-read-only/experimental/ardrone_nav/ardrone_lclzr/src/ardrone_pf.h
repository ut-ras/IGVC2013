#ifndef __ARDRONE_PF_H
#define __ARDRONE_PF_H

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "ardrone_particle.h"
#include <ar_recog/Tag.h>
#include <ar_recog/Tags.h>
#include "tagserver.h"
#include "ardrone_state.h"
#include "quat_utils.h"

/*
#define _ODOM_VEL_X_ERROR 0.028
#define _ODOM_VEL_Y_ERROR 0.03375
#define _ODOM_RPY_ERROR 0.0444
*/

/*
#define _ODOM_VEL_X_ERROR 1.2
#define _ODOM_VEL_Y_ERROR 1.2
#define _ODOM_RPY_ERROR 1.2
*/

#define _ODOM_VEL_X_ERROR 0.9
#define _ODOM_VEL_Y_ERROR 0.9
#define _ODOM_RPY_ERROR 0.9

#define _NUM_PARTICLES 150

using namespace std;

#define PI 3.1415926535

class ardrone_particle_filter
{

public:

  //Number of particles to be used
  unsigned int np;

  //standard deviation coefficient for linear velocity estimate from ARDrone
  btVector3 odom_linvel_error;

  //standard deviation coefficient for orientation estimate from ARDrone
  btVector3 odom_rpy_error;

  //standard deviation for measurement model (position) (meters)
  btVector3 measurement_model_pos_error;
  
  //standard deviation for measurement model (orientation) (radians)
  btVector3 measurement_model_rpy_error;

  // [0,2*PI]
  double yaw_min, yaw_max;

  //Collection of particles (orientation is [0,2*PI])
  vector<ardrone_particle> *vec_pf;

  ardrone_particle_filter()
    :np(_NUM_PARTICLES)
  {
    //the coeff here corresponds to a standard deviation of x% of linvel_x
    odom_linvel_error[0]=_ODOM_VEL_X_ERROR;
    //the coeff here corresponds to a standard deviation of x% of linvel_y    
    odom_linvel_error[1]=_ODOM_VEL_Y_ERROR;
    odom_linvel_error[2]=0.0;    

    odom_rpy_error[0]=0.0;
    odom_rpy_error[1]=0.0;
    //the coeff here corresponds to a standard deviation of x% of theta_z
    odom_rpy_error[2]=_ODOM_RPY_ERROR;

    yaw_min=0.0;
    yaw_max=0.0;

    measurement_model_pos_error[0]=0.5;
    measurement_model_pos_error[1]=0.5;
    measurement_model_pos_error[2]=0.0;

    measurement_model_rpy_error[0]=0.0;
    measurement_model_rpy_error[1]=0.0;
    measurement_model_rpy_error[2]=(60/180.0*PI);

    vec_pf = new vector<ardrone_particle>();
    vec_pf->resize(np);

  }
  
  ardrone_particle_filter(unsigned int nparg)
    :np(nparg)
  {
    //the coeff here corresponds to a standard deviation of x% of linvel_x
    odom_linvel_error[0]=_ODOM_VEL_X_ERROR;
    //the coeff here corresponds to a standard deviation of x% of linvel_y    
    odom_linvel_error[1]=_ODOM_VEL_Y_ERROR;
    odom_linvel_error[2]=0.0;    

    odom_rpy_error[0]=0.0;
    odom_rpy_error[1]=0.0;
    //the coeff here corresponds to a standard deviation of x% of theta_z
    odom_rpy_error[2]=_ODOM_RPY_ERROR;

    yaw_min=0.0;
    yaw_max=0.0;

    measurement_model_pos_error[0]=0.5;
    measurement_model_pos_error[1]=0.5;
    measurement_model_pos_error[2]=0.0;

    measurement_model_rpy_error[0]=0.0;
    measurement_model_rpy_error[1]=0.0;
    measurement_model_rpy_error[2]=(60/180.0*PI);

    vec_pf = new vector<ardrone_particle>();
    vec_pf->resize(np);

  }

  void set_error_parameters(const btVector3& linvel_err, const btVector3& rpy_err)
  {
    odom_linvel_error=linvel_err;
    odom_rpy_error=rpy_err;
  }

  void initialize_particles(const btVector3& init_pos, const btVector3& init_rpy)
  {
    yaw_min=init_rpy[2];
    yaw_max=init_rpy[2];

    vector<ardrone_particle>::iterator pf_it = vec_pf->begin();
    for(;pf_it!=vec_pf->end();pf_it++)
      {
	(*pf_it).w_pos=init_pos;
	(*pf_it).drone_rpy=init_rpy;
      }
  }

  void perform_motion_update(ardrone_state& drone_state);

  double custom_compute_gaussian_density(double rv, double sd);

  void perform_measurement_update(const ar_recog::Tags::ConstPtr& tagsmsg, vector<tagPose> &tagposes, double avg_posx, double avg_posy, const ros::Publisher &artag_visualization);

};

#endif


