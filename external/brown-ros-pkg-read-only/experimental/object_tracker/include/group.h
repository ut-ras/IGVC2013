/*
   Blob Group Message
   
   Jihoon
 */

#ifndef _GROUPP_H_
#define _GROUPP_H_

#include <ros/ros.h>

struct XY{
  int x;
  int y;

  XY(int _x,int _y) : x(_x),y(_y) {}
};

class Group
{
  public:
   Group() { size = 0; gNum = 0; points.clear(); sumX =0; sumY = 0; }
    void reset(int n) { size = 0; gNum = n; points.clear(); sumX =0; sumY = 0; }
    void add(int x,int y) { points.push_back(XY(x,y)); size++; sumX += x; sumY += y;  }
    void setMean() { meanX = (unsigned int)(sumX / size); meanY = (unsigned int)(sumY / size);}
    int getSize() { return size; }
    int getgNum() { return gNum; }
    int getMeanX() { return meanX; } 
    int getMeanY() { return meanY; } 

  private:
    std::vector<XY> points;
    int gNum;
    int size;
    unsigned int sumX;
    unsigned int sumY;
    unsigned int meanX;
    unsigned int meanY; 

    friend bool compGroup(const Group a,const Group b);

};

bool compGroup(const Group a,const Group b) { return a.size > b.size; }

#endif
