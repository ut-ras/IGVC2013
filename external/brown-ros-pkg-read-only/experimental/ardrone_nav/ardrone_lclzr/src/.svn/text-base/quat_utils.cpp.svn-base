#include "quat_utils.h"
#include <iostream>

#ifndef PI
#define PI 3.1415926535
#endif

using namespace std;

//works only if the quaternion represents a pure yaw rotation (yaw axis is the z axis)
double getQuaternionAngleRad(btQuaternion qtarg)
{
  double th = qtarg.getAngle();

  if(fabs(th)>(2*PI))
    {
      cout << "Bad value returned by qtarg.getAngle, th: " << th << "\n";
      return -9999;
    }

  /*
  if((fabs(qtarg.z())>(1e-6))&&(fabs(qtarg.w())>(1e-6)))
    {
      if((qtarg.z()/qtarg.w())<0)
	{
	  //the angle is in [-PI,0]
	  //converting it to [0,2*PI]
	  th=(th+2*PI);
	}
    }
  */

  /*
  if((qtarg.z()<0)||(qtarg.w()<0))
    {
      //the angle is in [-PI,0]
      //converting it to [0,2*PI]
      th=(th+2*PI);
    }
  */

  return th;
}

//works only if the quaternion represents a pure yaw rotation (yaw axis is the z axis)
double getQuaternionAngleDeg(btQuaternion qtarg)
{
  double th = qtarg.getAngle();

  if(fabs(th)>(2*PI))
    {
      cout << "Bad value returned by qtarg.getAngle, th: " << th << "\n";
      return -9999;
    }

  /*
  if((fabs(qtarg.z())>(1e-6))&&(fabs(qtarg.w())>(1e-6)))
    {
      if((qtarg.z()/qtarg.w())<0)
	{
	  //the angle is in [-PI,0]
	  //converting it to [0,2*PI]
	  th=(th+2*PI);
	}
    }
  */

  /*
  if((qtarg.z()<0)||(qtarg.w()<0))
    {
      //the angle is in [-PI,0]
      //converting it to [0,2*PI]
      th=(th+2*PI);
    }
  */

  double dval = (th/PI*180.0);
  return dval;
}

//q1 and q2 must always have values within [0,2*PI]
btQuaternion getQuaternionDiff(btQuaternion q1, btQuaternion q2)
{
  double th1 = q1.getAngle();
  double th2 = q2.getAngle();

  double diffval = (th1-th2);

  /*
  if((diffval<(-PI))&&((th1<(PI/2.0))&&(th1>0.0))&&((th2>(3*PI/2.0))&&(th2<(2*PI))))
    {
      diffval = (diffval+2*PI);
    }
  else if((diffval>PI)&&((th2<(PI/2.0))&&(th2>0.0))&&((th1>(3*PI/2.0))&&(th1<(2*PI))))
    {
      diffval = (diffval-2*PI);
    }
  */

  if(diffval<0)
    diffval=(diffval+2*PI);

  btQuaternion diffq;
  diffq.setRPY(0.0,0.0,diffval);

  return diffq;
}

//q1 and q2 must always have values within [0,2*PI]
//strong assumption here that q1 and q2 are not displaced by much from each other
double getQuaternionDiffShortestRad(btQuaternion q1, btQuaternion q2)
{
  double th1 = q1.getAngle();
  double th2 = q2.getAngle();

  double diffval = (th1-th2);
  ///double diffval1 = (th1-th2);

  /*
  cout << "th1: " << (th1/PI*180.0) << "\n";
  cout << "th2: " << (th2/PI*180.0) << "\n";
  */

  if((diffval<(-PI))&&((th1<=(PI/2.0))&&(th1>=0.0))&&((th2>=(3*PI/2.0))&&(th2<=(2*PI))))
    {
      diffval = (diffval+2*PI);
    }
  else if((diffval>PI)&&((th2<=(PI/2.0))&&(th2>=0.0))&&((th1>=(3*PI/2.0))&&(th1<=(2*PI))))
    {
      diffval = (diffval-2*PI);
    }

  ///cout << "2.diffval: " << (diffval/PI*180.0) << "\n";

  /*
  if(diffval<(-PI))
    diffval=(diffval+2*PI);
  else if(diffval>PI)
    diffval=(diffval-2*PI);
  */

  ///cout << "diffval: " << (diffval/PI*180.0) << "\n";

  return diffval;
}


