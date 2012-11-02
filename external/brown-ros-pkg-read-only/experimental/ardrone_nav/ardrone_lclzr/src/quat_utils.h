#ifndef _QUAT_UTILS_H_
#define _QUAT_UTILS_H_

#include <LinearMath/btQuaternion.h>

//works only if the quaternion represents a pure yaw rotation (yaw axis is the z axis)
double getQuaternionAngleRad(btQuaternion qtarg);

//works only if the quaternion represents a pure yaw rotation (yaw axis is the z axis)
double getQuaternionAngleDeg(btQuaternion qtarg);

//q1 and q2 must always have values within [0,2*PI]
btQuaternion getQuaternionDiff(btQuaternion q1, btQuaternion q2);

//q1 and q2 must always have values within [0,2*PI]
//strong assumption here that q1 and q2 are not displaced by much from each other
double getQuaternionDiffShortestRad(btQuaternion q1, btQuaternion q2);
#endif



