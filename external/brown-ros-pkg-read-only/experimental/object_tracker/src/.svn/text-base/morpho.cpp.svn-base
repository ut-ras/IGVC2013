
#include "morpho.h"
#include <stdio.h>
#include <ros/ros.h>
#include <cstdlib>
#include <boost/thread.hpp>

Morpho::Morpho()
{
  // everything is not in use
  // creates predefined masks
  masks = new bool*[NUM_MASK];

  maskSize[0] = 5;
  masks[0] = createMask(5);
  maskSize[1] = 19;
  masks[1] = createMask(19);
  maskSize[2] = 9;
  masks[2] = createMask(9);
}

Morpho::~Morpho()
{
  int i;
  for(i =0;i<NUM_MASK; i++)
    delete masks[i];

  delete masks;

}

// not in use
bool* Morpho::createMask(unsigned int size)
{
  bool* m = new bool[size * size];
  
  unsigned int i,j,index;
  unsigned int diff;
  int ctr_x,ctr_y;

  ctr_x = ctr_y = (int)size/2;

  for(i=0;i<size; i++)
  {
    for(j =0;j<size; j++)
    {
      index = i * size + j;

      diff = abs(i - ctr_x) + abs(j - ctr_x);
      m[index] = (diff <= (size/2))?1:0;
    }
  }
  return m;
}

void Morpho::dilate(bool* img,int w,int h,int size)
{
  int numThread = 4;
  int slice = h / numThread;
  int i;
  bool newimg[w * h];
  boost::thread t[numThread];

  for(i=0;i <numThread; i++) {
    t[i] = boost::thread(boost::bind(&Morpho::doit,this,newimg,img,i * slice,(i+1)*slice,w,h,size,DILATE_VAL));
  }
  for(i=0;i <numThread; i++) {
    t[i].join();
  }

  for(i = 0;i  < w * h; i++)
    img[i] = newimg[i];
}

void Morpho::erode(bool* img,int w,int h,int size)
{
  int numThread = 4;
  int slice = h / numThread;
  int i;
  bool newimg[w * h];
  boost::thread t[numThread];

  for(i=0;i <numThread; i++) {
    t[i] = boost::thread(boost::bind(&Morpho::doit,this,newimg,img,i * slice,(i+1)*slice,w,h,size,ERODE_VAL));
  }
  for(i=0;i <numThread; i++) {
    t[i].join();
  }

  for(i = 0; i < w * h; i++)
    img[i] = newimg[i];
}

void Morpho::doit(bool newimg[],bool* img,int s1,int s2,int w,int h,int size,int type)
{
  int x,y;
  int index;

  for(y =s1; y < s2; y++)
  {
    for(x=0; x < w; x++)
    {
      index = y * w + x; 
      newimg[index] = check(img,x,y,w,h,size,type);
    }
  }
}

bool Morpho::check(bool* img,int x,int y,int w,int h,int size,int type)
{
  int half = size/2;
  int st_x = x - half;
  int fi_x = x + half;
  int st_y = y - half;
  int fi_y = y + half;

  st_x = st_x<0?0:st_x;
  st_y = st_y<0?0:st_y;
  fi_x = fi_x >=w?w-1:fi_x;
  fi_y = fi_y >=h?h-1:fi_y;
  int s = (fi_x - st_x) * (fi_y - st_y);


  int i,j;
  int index;
  int sum = 0;

  for(i = st_y; i <= fi_y; i++) {
    for(j = st_x; j <= fi_x; j++) {
      index = i * w +j;

      sum += img[index];
    }
  }
//  std::cout << "y = " << y << "\tx = " << x << " sum = " << sum << std::endl;
//  sleep(0.5);

  if(type == ERODE_VAL)
    return sum > (s-1)?1:0;
  else if(type == DILATE_VAL) {
    return sum>0?1:0;
  }
  else {
    std::cerr << "ERROR!!!" << std::endl;
    return 0;
  }
}

// for debugging
/*
int main(int argc,char** argv)
{
  Morpho* m = new Morpho();

  int i,j;
  int index;
  int kk = 0;
  int size = m->maskSize[kk];
  for(i = 0 ; i <size; i++) {
    for(j = 0; j < size; j++) {
      index = i * size + j;
      printf("%d ",m->masks[kk][index]);
    }
    printf("\n");
  }

  return 0;
}*/
