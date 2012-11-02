/*
   Matrix Library

   Jihoon Lee
   12.2011
 */

#ifndef _MATRIX2x2_H_
#define _MATRIX2x2_H_

#include <ros/ros.h>

struct Float2 {
  Float2() { d[0] = 0; d[1] = 0; isCol = false;}
  Float2(float a,float b) { d[0] = a; d[1] = b; isCol = false;}
  void print() { std::cout << "(" << d[0] << ", " << d[1] << ")" << std::endl; }
  float d[2];
  bool isCol;
};

//template <class T>
class Matrix2x2 {
  public :
    Matrix2x2();
    Matrix2x2(float d[4]);
    Matrix2x2(float a,float b,float c,float d);
    ~Matrix2x2() {}
    void set(float d[4]);
    void set(float a,float b,float c,float d);
    void get(float& a,float& b,float& c,float& d);

    Matrix2x2 operator + (Matrix2x2& a);
    Matrix2x2 operator += (Matrix2x2& a);
    Matrix2x2 operator - (Matrix2x2& a);
    Matrix2x2 operator -= (Matrix2x2& a);
    Matrix2x2 operator * (Matrix2x2& a);
    Matrix2x2 operator * (float a);
    Float2 operator * (Float2 a);
    Float2 mult(Float2 a,int flag);
    Matrix2x2 transpose();
    Matrix2x2 inverse();
    float det();
    void print();
    
  private :
    float data[4]; // [0 1
                   //  2 3]
};

#endif
