#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
: H(1,0)
{
  sig_a = 0.5;
}

KalmanFilter::KalmanFilter(float x,float y)
: H(1,0)
{
  post_cov = Matrix2x2(1,0,0,1);
  post_pose = Matrix2x2(x,y,0,0);

  sig_a = 0.5;

//  t = ros::Time::now();
}

KalmanFilter::~KalmanFilter()
{
}
/*
KalmanFilter operator = (const KalmanFilter& k)
{
  H = k.H;
  post_cov = k.post_cov;
  post_pose = k.post_pose;
  sig_a = k.sig_a;
}
*/
void KalmanFilter::update(object_tracker::Blobgroup group,float dt)
{
  Float2 Y(group.pose[0].position.x,group.pose[0].position.y);

  // Prediction
  Matrix2x2 F(1,dt,0,1);
  Float2 B((dt *dt)/2,dt);
  Matrix2x2 Q(B.d[0] * B.d[0],B.d[0] * B.d[1], B.d[1] * B.d[0], B.d[1] * B.d[1]);
  Q = Q * sig_a;

  Matrix2x2 ftran = F.transpose();
  pre_pose = F * post_pose + Q;

  pre_cov = (F * post_cov) * ftran + Q;
  std::cout << "dt = " << dt << std::endl;
  std::cout << "Y = (" << Y.d[0] << ", " << Y.d[1] << ")" << std::endl;

  /*
  std::cout << "F =" << std::endl;
  F.print();

  std::cout << "B= " << std::endl;
  B.print();

  std::cout << "Q = " << std::endl;
  Q.print();
  */

  std::cout << "Prepose" << std::endl;
  pre_pose.print();
  /*
  std::cout << "Precov" << std::endl;
  pre_cov.print();
*/
  // Correction
  Float2 K;
  Float2 tmp;
  float t;
  tmp = pre_cov * H;
  t = tmp.d[0] * H.d[0] + tmp.d[1] * H.d[1] + 1;
  t = 1/t;

  K.d[0] = tmp.d[0] * t;
  K.d[1] = tmp.d[1] * t;

  tmp = pre_pose.mult(H,1);


  tmp.d[0] = Y.d[0] - tmp.d[0];
  tmp.d[1] = Y.d[1] - tmp.d[1];

  Matrix2x2 tt(K.d[0] * tmp.d[0],K.d[0] * tmp.d[1],
               K.d[1] * tmp.d[0],K.d[1] * tmp.d[1]);
  post_pose = pre_pose + tt;

  Matrix2x2 eye(1,0,0,1);
  tt = Matrix2x2(K.d[0] * H.d[0],K.d[0] * H.d[1],
                 K.d[1] * H.d[0],K.d[1] * H.d[1]);
  post_cov = (eye - tt) * pre_cov;
/*
  std::cout << "Kalman gain" << std::endl;
  K.print();
  */
  std::cout << "Postpose" << std::endl;
  post_pose.print();
  /*
  std::cout << "Postcov " << std::endl;
  post_cov.print();
  std::cout << std::endl << std::endl;
  */
}

object_tracker::Blobgroup KalmanFilter::getNewState()
{
  object_tracker::Blobgroup bg;

  return bg;
}

void KalmanFilter::predict(float dt)
{
  Matrix2x2 F(1,dt,0,1);
  Float2 B((dt *dt)/2,dt);
  Matrix2x2 Q(B.d[0] * B.d[0],B.d[0] * B.d[1], B.d[1] * B.d[0], B.d[1] * B.d[1]);
  Q = Q * sig_a;

  Matrix2x2 ftran = F.transpose();
  pre_pose = F * post_pose + Q;

  pre_cov = (F * post_cov) * ftran + Q;
}

void KalmanFilter::correct(float nx,float ny)
{
  Float2 Y(nx,ny);
  // Correction
  Float2 K;
  Float2 tmp;
  float t;
  tmp = pre_cov * H;
  t = tmp.d[0] * H.d[0] + tmp.d[1] * H.d[1] + 1;
  t = 1/t;

  K.d[0] = tmp.d[0] * t;
  K.d[1] = tmp.d[1] * t;

  tmp = pre_pose.mult(H,1);


  tmp.d[0] = Y.d[0] - tmp.d[0];
  tmp.d[1] = Y.d[1] - tmp.d[1];

  Matrix2x2 tt(K.d[0] * tmp.d[0],K.d[0] * tmp.d[1],
               K.d[1] * tmp.d[0],K.d[1] * tmp.d[1]);
  post_pose = pre_pose + tt;

  Matrix2x2 eye(1,0,0,1);
  tt = Matrix2x2(K.d[0] * H.d[0],K.d[0] * H.d[1],
                 K.d[1] * H.d[0],K.d[1] * H.d[1]);
  post_cov = (eye - tt) * pre_cov;
}

geometry_msgs::Pose KalmanFilter::getPose()
{
  geometry_msgs::Pose p;
  float x,y,vx,vy;

  post_pose.get(x,y,vx,vy);

  p.position.x = x;
  p.position.y = y;

  return p;
}

/*
int main(int argc,char** argv)
{
  KalmanFilter* kf = new KalmanFilter(0,0);
  object_tracker::Blobgroup bg;

  bg.pose.resize(1);
  bg.pose[0].position.x = 136;
  bg.pose[0].position.y = 376;

  kf->update(bg,0.2018757);

  bg.pose[0].position.x = 135;
  bg.pose[0].position.y = 376;

  kf->update(bg,0.350035);

  bg.pose[0].position.x = 135;
  bg.pose[0].position.y = 376;

  kf->update(bg,0.460154);

  bg.pose[0].position.x = 135;
  bg.pose[0].position.y = 376;

  kf->update(bg,0.552555);

  delete kf;
  return 0;

}*/
