#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

//#include "motion_sample.h"
#include "pf/pf_pdf.h"

class ardrone_particle
{
 public:
  //Drone position in world coordinates
  btVector3 w_pos;
  
  //Drone linear velocity in world coordinates
  btVector3 w_linvel;
  
  //Drone orientation - 
  //This gives the XYZ Euler angle rotation matrix as per Craig's book pg 442 Appendix B
  //phi or drone_rpy[0] is the roll about the body fixed X axis
  //theta or drone_rpy[1] is the pitch about the body fixed Y axis
  //psi or drone_rpy[2] is the yaw about the body fixed Z axis
  btVector3 drone_rpy;

  //local sequence counter
  unsigned int seqcnt;

  //Particle importance weight
  double wt;
  
 ardrone_particle():w_pos(0,0,0),w_linvel(0,0,0),drone_rpy(0,0,0),seqcnt(0),wt(1) {}
  
 ardrone_particle(const btVector3& pos, const btVector3& rpy, double wtarg=1.0)
   :w_pos(pos),w_linvel(0,0,0),drone_rpy(rpy),seqcnt(0),wt(wtarg) {}

  ardrone_particle(const ardrone_particle &a)
    {
      w_pos=a.w_pos;
      w_linvel=a.w_linvel;
      drone_rpy=a.drone_rpy;
      seqcnt=a.seqcnt;
      wt=a.wt;
    }

  void particle_initialize(const btVector3& pos, const btVector3& vel, const btVector3& rpy, double wtarg=1.0)
  {
    wt=wtarg;
    w_pos=pos;
    w_linvel=vel;
    drone_rpy=rpy;
  }
};



