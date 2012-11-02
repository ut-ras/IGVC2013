#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <termios.h>
#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include "get_ik.cpp"
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>

using namespace std;

struct reqCartesianStruct{
  float px;
  float py;
  float pz;
  float ox;
  float oy;
  float oz;
  float ow;
};

struct blobData{
  int area;
  int x;
  int y;
};

struct reqCartesianStruct ikRequest;
struct blobData rightBlob;
struct blobData leftBlob;
int image1grab;
int image2grab;


//handles incoming blob image messages
/*void right_blob_handler(const cmvision::Blobs& msg)
{
  if(msg->blob_count=0){
    return;
  }
  int largearea=0;
  int largeblob=0;
  for (int i=0; i<msg->blob_count; i++)
    {
      cmvision::Blob currblob=msg->blobs(i);
      if(msg->blobs(i)->area >largearea)
	  largeblob=i;
	
    }
  
  rightBlob.area=blobs(i)->area;
  rightBlob.x=blobs(i)->x;
  rightBlob.y=blobs(i)->y;

}

void left_blob_handler(const cmvision::Blobs& msg)
{
 if(msg->blob_count=0){
    return;
  }
  int largearea=0;
  int largeblob=0;
  for (int i=0; i<msg->blob_count; i++)
    {
      cmvision::Blob currblob=msg->blobs(i);
      if(msg->blobs(i)->area >largearea)
	  largeblob=i;
	
    }
  
  leftBlob.area=msg->blobs(i)->area;
  leftBlob.x=blobs(i)->x;
  leftBlob.y=blobs(i)->y;

}

void calculate_position()
{

}*/

int main(int argc, char **argv){
 
  ros:: init(argc, argv, "object_reach");
  ros::NodeHandle n;
   ros::Rate loop_rate(30);
  //  ros::Subscriber right_blobs_sub=n.subscribe("right_blobs", 1, right_blob_handle);
  // ros::Subscriber left_blobs_sub=n.subscribe("left_blobs", 1, left_blob_handle);

  //move
  double request[7];

  request[0]=1.12;
  request[1]=0;
  request[2]=.78;
 


  double* poisition=getIK(3, request);

  ros::spinOnce();
  loop_rate.sleep();

  ros::shutdown();
  return 1;
}
