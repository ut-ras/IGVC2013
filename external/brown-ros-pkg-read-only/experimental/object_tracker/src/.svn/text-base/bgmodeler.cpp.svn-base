#include "bgmodeler.h"

BGModeler::BGModeler(ros::NodeHandle& nh,int num):
  node(nh), it(nh), numSet(num)
{
  node.param("bgmodel_out",bgm_out,std::string("bg.dat"));
  topic = "image";
  isFirst = true;
  isReady = false;
  cnt = 0;

  bgm = new BGModel();
}

BGModeler::~BGModeler()
{
  delete bgm;
}

void BGModeler::spin()
{
  std::cout << "topic = " << topic << std::endl;
  std::cout << "Number of image = " << numSet << std::endl;
  sub = it.subscribe(topic, 1, &BGModeler::imageCallback, this); 

  ros::Rate r(100);
  while(isReady == false && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  bgm->save(bgm_out);
}


void BGModeler::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // it assumes that the size of image doesn't change while collecting
  if(isFirst)
  {
    std::cout << "Starting Sampling" << std::endl;
    cnt = 0;
    bgm->width = (int)msg->width;
    bgm->height = (int)msg->height;

    bgm->mean = new float[bgm->width * bgm->height];
    bgm->variance = new float[bgm->width * bgm->height];

    int i;
    for(i = 0; i< bgm->width* bgm->height;i++) {
      bgm->mean[i] = 0;
      bgm->variance[i] = 0;
    }

    isFirst = false;
  }
  else if(bgm->width != (int)msg->width || bgm->height != (int)msg->height)
  {
    ROS_ERROR("Size of width or height has been changed..");
    isFirst = true;
    cnt = 0;
    delete bgm->mean;
    delete bgm->variance;
    return;
  }
  else if(cnt < numSet) {
    int i,j,k;
    unsigned char r,g,b;
    float gs;
    int rowLength = msg->step;
    int index;
  
//    std::cout << "step = " << msg->step << std::endl;
//    std::cout << "rowLength = " << rowLength << std::endl;
    cnt++;
    k = 0;
    for(j = 0; j < bgm->height; j++)
    {
      for(i = 0; i < rowLength;i+=3)
      {
        index = rowLength * j + i;
        r = (unsigned char)(msg->data[index]);
        g = (unsigned char)(msg->data[index+1]);
        b = (unsigned char)msg->data[index+2];

        gs = COMPUTE_GRAYSCALE(r,g,b);
        //std::cout << "gs = " << (int)gs << std::endl;
        bgm->mean[k] += (float)((float)gs/(float)numSet);
        bgm->variance[k] += (float)((gs * gs)/(float)numSet);
      
        k++;
      }
    }
  }
  else {
    std::cout << "Done Sampling" <<std::endl;
    std::cout << "numSet = " << numSet << std::endl;
    
    int i;
    for(i = 0; i < bgm->width* bgm->height; i++)
    {
     bgm->variance[i] = bgm->variance[i] - (bgm->mean[i] * bgm->mean[i]);
    }

    std::cout << "bgm->mean[0] = " << bgm->mean[0] << std::endl;
    std::cout << "bgm->variance[0] = " << bgm->variance[0] << std::endl;

    isReady = true;
  }
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"bgmodeler");
  if (ros::names::remap("image") == "image") {
    ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun objecr_tracker bgmodeler image:=<image topic>");
  }

  int numSet = 10;
  if(argc == 2)
  {
    numSet = atoi(argv[1]);  
  }

  ros::NodeHandle nh;

  BGModeler* modeler;
  modeler = new BGModeler(nh,numSet);
  printf("BackgroundModeler initialized\n");

  modeler->spin();

  delete modeler;

  return 0;
}
