
/*
  BGModel

  Background Model. Each pixel stores as gaussian distribution

  Jihoon Lee
  12.2011
*/

#ifndef _BGMODEL_H_
#define _BGMODEL_H_

#include <ros/ros.h>
#include <fstream>

// for grayscale
#define RED_V 0.299f
#define GREEN_V 0.587f
#define BLUE_V 0.114f
 
#define COMPUTE_GRAYSCALE(x,y,z) ({ (unsigned char)(RED_V * (float)x + GREEN_V * (float)y + BLUE_V * (float)z); })

class BGModel {
  public:
    BGModel();
    ~BGModel();
    void load(std::string fileName);
    void save(std::string fileName);

    int width;
    int height;
    float* mean;
    float* variance;
};

#endif // _BGMODEL_H_
