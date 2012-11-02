#ifndef __TAGSERVER_H
#define __TAGSERVER_H

#include <vector>

using namespace std;

class tagPose
{
public:
  //tag numerical id
  unsigned int id;
  double x;
  double y;
  //Tag rotation with respect to world frame
  double trot;
  tagPose()
  {
    id=0;
    x=0.0;
    y=0.0;
    trot=0.0;
  }
  tagPose(unsigned int idarg, double xarg, double yarg, double rotarg)
  {
    id=idarg;
    x=xarg;
    y=yarg;
    trot=rotarg;
  }
};


/*
tagPose* findtagpose(unsigned int idarg)
{
  vector<tagPose>::iterator it=tagposes.begin();
  for(;it<tagposes.end();it++)
    if((*it).id==idarg) break;
  return &(*it);
}
*/

#endif
