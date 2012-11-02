#include "object_tracker.h"

ObjectTracker::ObjectTracker(ros::NodeHandle& n) :
  node(n), it(n)
{
  subtopic = "blobgroup_raw";
  pubtopic = "blobgroup";
  t = ros::Time::now();
  objNum = 1;
}

ObjectTracker::~ObjectTracker() {
}

void ObjectTracker::msgCallback(const object_tracker::Blobgroup& msg)
{
  object_tracker::Blobgroup newmsg = msg;
  float dt = (msg.header.stamp - t).toSec();
  unsigned int newsize = newmsg.pose.size();
  unsigned int oldsize = gkf.size();
  unsigned int i;
  int idx;
  bool chk[newsize];

  for(i=0; i < newsize; i++)
    chk[i] = false;

  // prediction
  for(i=0; i <oldsize;i++)
    gkf[i].kf.predict(dt);

//  std::cout << "old size =  " << oldsize << std::endl;
//  std::cout << "new size =  " << newsize << std::endl;
  
  for(i=0; i < oldsize; i++) {
    // get matching blob
    getMatchingBlob(gkf[i].kf.getPose(),newmsg.pose,idx,chk); 

    if(idx >= 0) // if matching blob is found, update kalman filter
    {
      float nx = newmsg.pose[idx].position.x;
      float ny = newmsg.pose[idx].position.y;
      gkf[i].size = newmsg.blob_size[idx];
      gkf[i].kf.correct(nx,ny);
    }
    else // if matching blob is not found, erase object
    {
      gkf.erase(gkf.begin() + i);
    }
  }

  // create new object 
  for(i =0; i < newsize; i++)
  {
    if(!chk[i])
    {
      unsigned int n = objNum++;
      KalmanFilter filter(newmsg.pose[i].position.x,newmsg.pose[i].position.y);
      GroupKF gk(n,newmsg.blob_size[i],filter);
      
      gkf.push_back(gk);
    }
  }
  
  t = msg.header.stamp;

  publishObjects();
}

void ObjectTracker::getMatchingBlob(geometry_msgs::Pose oldp,std::vector<geometry_msgs::Pose> newp,int& idx,bool chk[])
{ 
  float mindist = -1;
  int minidx = -1;
  float dist;
  unsigned int i; 
  geometry_msgs::Point a;
  geometry_msgs::Point b;
  
  for(i = 0; i < newp.size(); i++) {
    a = oldp.position;
    b = newp[i].position;
    dist =  sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y) *(a.y -b.y) + (a.z - b.z) * (a.z - b.z));

    if(mindist == -1 || dist < mindist) {
      mindist = dist;
      minidx = i;
    }
  }
  
  if(mindist > 50 || minidx < 0) {
    idx = -1;
  }
  else {
    idx = minidx;
    chk[idx] = true;
  }
}

void ObjectTracker::publishObjects()
{
  object_tracker::Blobgroup m;
  unsigned int size = gkf.size();
  unsigned int i;
  geometry_msgs::Pose p;

  m.header.stamp = ros::Time::now();
  m.group_num.resize(size);
  m.blob_size.resize(size);
  m.pose.resize(size);

  for(i =0; i < size; i++)
  {
    m.group_num[i] = gkf[i].gNum;
    m.blob_size[i] = gkf[i].size;
    m.pose[i] = gkf[i].kf.getPose();
  }

  pub.publish(m);
}
    
void ObjectTracker::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::Image newmsg;
  float R = 40;
  float x,y;
  int st_x,st_y;
  int fi_x,fi_y;
  int st,fi;
  unsigned int i,j;
  int width;
  int height;
  unsigned int idx;

  // background subtraction
  newmsg.header = msg->header;
  width=newmsg.width = msg->width;
  height = newmsg.height = msg->height;
/*  
  newimg.encoding = msg->encoding;
  newimg.is_bigendian = msg->is_bigendian;
  newimg.step = msg->step;
*/
  newmsg = *msg;

  for(j = 0; j < gkf.size();j++)
  {
    geometry_msgs::Pose p = gkf[j].kf.getPose();
    x = p.position.x;
    y = p.position.y;

    st_x = (int)x - R;
    fi_x = (int)x + R;
    st_y = (int)y - R;
    fi_y = (int)y + R;

    // top line
    if(st_y > 0) {
//      st = st_x < 0 ?0:st_x;
//      fi = fi_x >= width?width-1:fi_x;
      st = st_x * 3;
      fi = fi_x >= width?msg->step-3:fi_x*3;
      for(i=st; i <fi; i+=3) {
        idx = (st_y * msg->step) + i;
        newmsg.data[idx] = 0;
        newmsg.data[idx+1] = 0;
        newmsg.data[idx+2] = 255;
      }
    }
    // botom line
    if(fi_y < height)
    {
      st = st_x*3 <0 ?0:st_x*3;
      fi = fi_x >= width?msg->step-3:fi_x*3;
      for(i=st; i <fi; i+=3) {
        idx = fi_y * msg->step + i;
        newmsg.data[idx] = 0;
        newmsg.data[idx+1] = 0;
        newmsg.data[idx+2] = 255;
      }
    }
    // left line
    if(st_x >= 0)
    {
      st = st_y < 0 ? 0:st_y;
      fi = fi_y >= height?height-1:fi_y;
      for(i = st; i < fi; i ++) {
        idx = i * msg->step + st_x * 3;
        newmsg.data[idx] = 0;
        newmsg.data[idx+1] = 0;
        newmsg.data[idx+2] = 255;
      }
    }

    if(fi_x < height)
    {
      st = st_y < 0 ? 0:st_y;
      fi = fi_y >= height?height-1:fi_y;
      for(i = st; i < fi; i ++) {
        idx = i * msg->step + fi_x * 3;
        newmsg.data[idx] = 0;
        newmsg.data[idx+1] = 0;
        newmsg.data[idx+2] = 255;
      }
    }
  }

  pub2.publish(newmsg);
}


void ObjectTracker::spin()
{
  pub = node.advertise<object_tracker::Blobgroup>(pubtopic, 5);
  sub = node.subscribe(subtopic,1, &ObjectTracker::msgCallback,this);
  sub2 = it.subscribe("image", 1, &ObjectTracker::imgCallback, this); 
  pub2 = it.advertise("final_track",1);

  ROS_INFO("Object Tracker initialized");
  ros::spin();
}

int main(int argc,char** argv) {
  
  ros::init(argc,argv,"object_tracker");

  ros::NodeHandle nh;

  ObjectTracker* tracker;
  tracker = new ObjectTracker(nh);

  tracker->spin();

  delete tracker;

  return 0;
}

