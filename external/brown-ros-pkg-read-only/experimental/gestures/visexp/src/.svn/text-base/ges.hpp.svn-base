#ifndef GES_H
#define GES_H

// syntactic sugar for dealing with IplImages.
#define pindex(arr,x,y) ((uchar *)(arr->imageData + y*arr->widthStep))[x]

#include <opencv/cv.h>
#include <opencv/highgui.h>

typedef struct var {
  int x;
  int y;
  int seg;
  int rX;
  int rY;
  int rWidth;
  int rHeight;
  int gesture;
  int streak;
} person_t;


bool processFrame(IplImage* raw, int ve_width, int ve_height, double m, double b, int ve_tol, int ve_minsize, person_t* person, IplImage* paint_img);

#endif
