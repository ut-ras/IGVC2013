#ifndef __ARDRONE_STATE_H
#define __ARDRONE_STATE_H

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

//#include "motion_sample.h"

class ardrone_state
{
 public:
  //Drone linear velocity in Drone body coordinates
  btVector3 b_lin_vel;

  double rpsi;
  double rpsi_prev;
  double rpsi_init;

  btQuaternion q_yaw;
  btQuaternion q_yaw_prev;
  btQuaternion q_yaw_init;

  //Time stamp from the drone in microseconds
  double tm;

  //delta_t in seconds
  double dt;

  //Initial offset of drone position from origin of world frame, represented in world frame
  btVector3 w_init_pos_offset;

  //local sequence counter
  unsigned int seqcnt;

  ardrone_state()
    {
      tm=0.0;
      dt=0.0;
      seqcnt=0;
      q_yaw.setRPY(0.0,0.0,0.0);
      q_yaw_prev.setRPY(0.0,0.0,0.0);
      q_yaw_init.setRPY(0.0,0.0,0.0);
      rpsi=0.0;
      rpsi_prev=0.0;
      rpsi_init=0.0;
      for(int i=0; i<3; i++)
	{
	  w_init_pos_offset[i]=0.0;
	  b_lin_vel[i]=0.0;
	}
    }

 ardrone_state(const btVector3& rpy, const btVector3& init_pos, \
	       const btVector3& init_rpy, double start_tm=0.0)
   :tm(start_tm)
  {
    dt=0.0;
    seqcnt=0;
    q_yaw.setRPY(0.0,0.0,rpy[2]);
    q_yaw_prev.setRPY(0.0,0.0,rpy[2]);
    q_yaw_init.setRPY(0.0,0.0,init_rpy[2]);
    rpsi=rpy[2];
    rpsi_prev=rpy[2];
    rpsi_init=init_rpy[2];
    for(int i=0; i<3; i++)
      {
	w_init_pos_offset[i]=init_pos[i];
	b_lin_vel[i]=0.0;
      }
  }

  void state_initialize(const btVector3& rpy, const btVector3& init_pos, \
			const btVector3& init_rpy, double start_tm=0.0)
  {
    dt=0.0;
    tm=start_tm;
    q_yaw.setRPY(0.0,0.0,rpy[2]);
    q_yaw_prev.setRPY(0.0,0.0,rpy[2]);
    q_yaw_init.setRPY(0.0,0.0,init_rpy[2]);
    rpsi=rpy[2];
    rpsi_prev=rpy[2];
    rpsi_init=init_rpy[2];
    for(int i=0; i<3; i++)
      {
	w_init_pos_offset[i]=init_pos[i];
	b_lin_vel[i]=0.0;
      }
  }

  void initialize_pose(const btVector3& init_pos, const btVector3& init_rpy)
  {
    rpsi=0.0;
    rpsi_prev=0.0;
    rpsi_init=init_rpy[2];
    q_yaw.setRPY(0.0,0.0,0.0);
    q_yaw_prev.setRPY(0.0,0.0,0.0);
    q_yaw_init.setRPY(0.0,0.0,init_rpy[2]);
    for(int i=0; i<3; i++)
      {
	w_init_pos_offset[i]=init_pos[i];
	b_lin_vel[i]=0.0;
      }
  }
};

#endif





