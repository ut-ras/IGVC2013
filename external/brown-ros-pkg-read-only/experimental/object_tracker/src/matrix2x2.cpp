
#include "matrix2x2.h"

Matrix2x2::Matrix2x2()
{
  int i;
  for(i=0;i<4;i ++)
    data[i] = 0;
}

Matrix2x2::Matrix2x2(float d[4])
{
  set(d);
}

Matrix2x2::Matrix2x2(float a,float b,float c,float d)
{
  set(a,b,c,d);
}

void Matrix2x2::set(float d[4])
{
  for(int i=0;i<4;i++)
    data[i] = d[i];

  return;
}

void Matrix2x2::set(float a,float b,float c,float d)
{
  data[0] = a;
  data[1] = b;
  data[2] = c;
  data[3] = d;
}

void Matrix2x2::get(float& a,float& b,float& c,float& d)
{
  a = data[0];
  b = data[1];
  c = data[2];
  d = data[3];
}

Matrix2x2 Matrix2x2::operator + (Matrix2x2& a)
{
  Matrix2x2 out;

  int i;
  for(i=0;i < 4; i++)
    out.data[i] = data[i] + a.data[i];

  return out;
}

Matrix2x2 Matrix2x2::operator += (Matrix2x2& a)
{
  Matrix2x2 out;

  int i;
  for(i=0;i < 4; i++)
    data[i] += a.data[i];

  return *this;
}

Matrix2x2 Matrix2x2::operator - (Matrix2x2& a)
{
  Matrix2x2 out;

  int i;
  for(i=0;i < 4; i++)
    out.data[i] = data[i] - a.data[i];

  return out;
}

Matrix2x2 Matrix2x2::operator -= (Matrix2x2& a)
{
  Matrix2x2 out;

  int i;
  for(i=0;i < 4; i++)
    data[i] -= a.data[i];

  return *this;
}

Matrix2x2 Matrix2x2::operator * (Matrix2x2& a)
{
  Matrix2x2 out;

  out.data[0] = data[0] * a.data[0] + data[1] * a.data[2];
  out.data[1] = data[0] * a.data[1] + data[1] * a.data[3];
  out.data[2] = data[2] * a.data[0] + data[3] * a.data[2];
  out.data[3] = data[2] * a.data[1] + data[3] * a.data[3];

  return out;
}

// assume a is column vector
Float2 Matrix2x2::operator * (Float2 a)
{
  Float2 out;

  out.d[0] = data[0] * a.d[0] + data[1] * a.d[1];
  out.d[1] = data[2] * a.d[0] + data[3] * a.d[1];

  return out;
}

Float2 Matrix2x2::mult(Float2 a,int flag)
{
  if(flag)
  {
    Float2 out;

    out.d[0] = a.d[0] * data[0] + a.d[1] * data[2];
    out.d[1] = a.d[0] * data[1] + a.d[1] * data[3];
    return out;
  }
  else
    return (*this) * a;
}


Matrix2x2 Matrix2x2::operator *(float a)
{
  Matrix2x2 out;
  int i;
  for(i=0; i < 4; i++)
  {
    out.data[i] = data[i] * a;
  }

  return out;
}

Matrix2x2 Matrix2x2::transpose()
{
  Matrix2x2 out;

  out.data[0] = data[0];
  out.data[1] = data[2];
  out.data[2] = data[1];
  out.data[3] = data[3];

  return out;
}

float Matrix2x2::det()
{
  return (data[0] * data[3]) - (data[1] * data[2]);
}

Matrix2x2 Matrix2x2::inverse()
{
  float d = det();
  Matrix2x2 out;

  if(d == 0) {
    std::cerr << "Determinant = 0" << std::endl;
    return out;
  }

  d = 1/d;
  out.data[0] = d * data[3];
  out.data[1] = -d * data[1];
  out.data[2] = -d * data[2];
  out.data[3] = d * data[0];

  return out;
}

void Matrix2x2::print()
{
  std::cout << data[0] << " " << data[1] << std::endl;
  std::cout << data[2] << " " << data[3] << std::endl;
}

/*
int main(int argc,char** argv)
{
  Matrix2x2 a(1,2,3,2);
  Matrix2x2 b(2,2,4,4);
  Matrix2x2 c;

  c = a + b;
  c.print();
  std::cout << std::endl;

  c = b - a;
  c.print();
  std::cout << std::endl;

  c = a * b;
  c.print();
  std::cout << std::endl;
  c = a.inverse();
  c.print();
  std::cout << std::endl;

  c = a.transpose();
  c.print();
  std::cout << std::endl;

  return 0;

}
*/
