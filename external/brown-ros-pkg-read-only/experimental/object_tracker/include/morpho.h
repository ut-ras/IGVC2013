/*
  Morphology filter

  it only takes binary image.
  Only uses box masks.

  // Only able to use predefined masks for now

  Jihoon Lee
  12.2011
 */

#ifndef _MORPHO_H_
#define _MORPHO_H_

#include <cstdlib>

// Predefined Mask
#define NUM_MASK 3 
#define DILATE_VAL 11
#define ERODE_VAL 12

class Morpho {
  public:
    Morpho();
    ~Morpho();
    void dilate(bool* img,int w,int h,int size); 
    void erode(bool* img,int w,int h,int size); 
    void doit(bool newimg[],bool* img,int s1,int s2,int w,int h,int size,int type); 
/*
    void open(bool[] img,int w,int h,int idx); 
    void close(bool[] img,int w,int h,int idx);*/
    bool* createMask(unsigned int size);
    bool check(bool* img,int i,int j,int w,int h,int size,int type);
//  private:
    bool** masks;
    unsigned int maskSize[NUM_MASK];
};

#endif // _MORPHO_H_
