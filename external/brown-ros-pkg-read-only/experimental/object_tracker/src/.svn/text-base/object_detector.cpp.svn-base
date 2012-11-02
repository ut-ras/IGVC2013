#include "object_detector.h"

ObjectDetector::ObjectDetector(ros::NodeHandle& n) :
  node(n), it(n)
{
  node.param("bgmodel",bgmodel_file,std::string("bg100.dat"));
  topic = "image";
  pubtopic = "tracker_image";
}

ObjectDetector::~ObjectDetector()
{
}

void ObjectDetector::spin()
{
  // Load backgroundmodel
  ROS_INFO("Background Model Loading..");
  bg.load(bgmodel_file);
  ROS_INFO("Background Model Loaded");
  
  sub = it.subscribe(topic, 1, &ObjectDetector::fgCallback, this); 
  pub = it.advertise(pubtopic,1);
  pub2 = it.advertise("morph_img",1);
  pub3 = node.advertise<object_tracker::Blobgroup>("blobgroup_raw",5);

  ROS_INFO("Object Detector initialized");
  ros::spin();
}

void ObjectDetector::fgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::Image newimg;
 // std::cout << "seq = " << msg->header.seq << std::endl;
  ros::Time t = ros::Time::now();
  ros::Time e;
  unsigned int size = msg->width * msg->height;
  bool bimg[size];
  int gr[size];

  group.clear();
  
  // get foreground images
  subtractBackground(newimg,bimg,msg);
  pub.publish(newimg);
/*

  e = ros::Time::now();
  t = e;
  // collect blobs
  morpho.dilate(bimg,msg->width,msg->height,2);
  morpho.erode(bimg,msg->width,msg->height,15);
  morpho.dilate(bimg,msg->width,msg->height,4);
  e = ros::Time::now();
  
//  std::cout << "morpho elapse      = " << e - t << std::endl;

  unsigned int i;

  // grouping the blobs
  grouping(bimg,gr,msg->width,msg->height);

//  std::cout << "after grouping     = " << e - t << std::endl;

  // blob size thresholding
  for(i=0; i < group.size(); i++)
    if(group[i].getSize() < 1500) {
      group.erase(group.begin() + i);
      i--;
    }
  std::sort(group.begin(),group.end(),compGroup);
//  output("out.dat",gr,msg->width,msg->height);

  publishBlobgroup();
  convertAndPub(newimg,msg,gr);*/
}

void ObjectDetector::publishBlobgroup()
{
  unsigned int i;
  object_tracker::Blobgroup bg;
  geometry_msgs::Pose p;

  unsigned int size = group.size();
  bg.group_num.resize(size);
  bg.pose.resize(size);
  bg.blob_size.resize(size);
  for(i=0; i < group.size(); i++)
  {
    bg.group_num[i] = group[i].getgNum();
    bg.blob_size[i] = group[i].getSize();
    p.position.x = group[i].getMeanX();
    p.position.y = group[i].getMeanY();
    bg.pose[i] = p;
  }

  bg.header.stamp = ros::Time::now();
  pub3.publish(bg);
}

void ObjectDetector::convertAndPub(sensor_msgs::Image& newimg,const sensor_msgs::ImageConstPtr& msg,int* gr)
{
  unsigned int i,j,k;
  unsigned int index,idx;
  for(j =0; j < newimg.height; j++) {
    for(k=0,i=0; i < newimg.step; i+=3,k++) {
      index = j * newimg.step + i;
      idx = j * newimg.width + k;
      if(gr[idx] == group[0].getgNum()) {
        newimg.data[index] = 255;
        newimg.data[index+1] = 0;
        newimg.data[index+2] = 0;
      }
      else if(group.size() > 1 && gr[idx] == group[1].getgNum()) {
        newimg.data[index] = 0;
        newimg.data[index+1] = 255;
        newimg.data[index+2] = 0;
      }
      else if(group.size() > 2 && gr[idx] == group[2].getgNum()) {
        newimg.data[index] = 0;
        newimg.data[index+1] = 0;
        newimg.data[index+2] = 255;
      }
      else if(group.size() > 3 && gr[idx] == group[3].getgNum()) {
        newimg.data[index] = 255;
        newimg.data[index+1] = 255;
        newimg.data[index+2] = 0;
      }
      else {
        newimg.data[index] = 0;msg->data[index];
        newimg.data[index+1] = 0;msg->data[index+1];
        newimg.data[index+2] = 0;msg->data[index+2];
      }
    }
  }

  pub2.publish(newimg);
}

void ObjectDetector::grouping(bool* img,int g[],int w,int h)
{
  int i,j;
  int index;
  int groupNum = 1;
  Group gr;

  for(i=0; i < w * h; i++)
    g[i] = 0;

  for(i = 0; i < h; i++)
  {
    for(j=0; j < w; j++) {
      index = i * w + j;

      if(img[index] == 1 && g[index] == 0) {
        gr.reset(groupNum);
        groupit(img,gr,g,i,j,w,h,groupNum);
        gr.setMean();

        group.push_back(gr);
        groupNum++;
      }
    }
  }
//  std::cout << groupNum << std::endl;
}

void ObjectDetector::groupit(bool img[],Group& group,int g[],int y,int x,int w,int h,int groupNum)
{
  static int ni[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
  int index = y * w + x;
  int nx,ny;

  group.add(x,y);
  g[index] = groupNum;

  int i;
  for(i = 0; i < 4; i++) {
    ny = y + ni[i][0];
    nx = x + ni[i][1];
    index = ny * w + nx;
    if(ny >= 0 && ny < h && nx >= 0 && nx < w && img[index] && g[index] != groupNum)
      groupit(img,group,g,ny,nx,w,h,groupNum);
  }
}

void ObjectDetector::output(char* filename,int* img,int w,int h)
{
  FILE* fp = fopen(filename,"w"); 
  int i;

  for(i=0; i < h * w;i++) {
    if(i % w == 0)
     fprintf(fp,"\n");

    fprintf(fp,"%d ",img[i]);
  }

  fclose(fp);
}


void ObjectDetector::subtractBackground(sensor_msgs::Image& newimg,bool bimg[],const sensor_msgs::ImageConstPtr& msg)
{
  int width,height;
  // background subtraction
  newimg.header = msg->header;
  width=newimg.width = msg->width;
  height = newimg.height = msg->height;
  
  newimg.encoding = msg->encoding;
  newimg.is_bigendian = msg->is_bigendian;
  newimg.step = msg->step;

  unsigned int i,j,k;
  unsigned int rowLength = msg->step;
  int index;
  int newidx;
  unsigned char r,g,b;
  float gs;
  unsigned int v;
  
  newimg.data.resize(rowLength * msg->height);
  
  k = 0;
  for(j = 0; j < msg->height; j++)
  {
    for(i = 0,k = 0; i < rowLength; i+=3,k++)
    {
      index = rowLength * j + i;
      newidx = msg->width * j + k;
      r = (unsigned char)(msg->data[index]);
      g = (unsigned char)(msg->data[index+1]);
      b = (unsigned char)msg->data[index+2];

      gs = COMPUTE_GRAYSCALE(r,g,b);
      v = (unsigned int)(abs((gs - bg.mean[newidx])));

      v = v < 20?0:255;
      newimg.data[index] = v;
      newimg.data[index+1] = v;
      newimg.data[index+2] = v;


      bimg[newidx] = (v==255)?1:0;
    }
  }
//  std::cout << "k " << k << std::endl;

  newimg.header.stamp = ros::Time::now();
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"object_detector");
  if (ros::names::remap("image") == "image") {
    ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun objecr_tracker bgmodeler image:=<image topic>");
  }

  ros::NodeHandle nh;

  ObjectDetector* detector;
  detector = new ObjectDetector(nh);

  detector->spin();

  delete detector;

  return 0;
}
