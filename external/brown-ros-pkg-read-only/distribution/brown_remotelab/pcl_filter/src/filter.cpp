#include"pc2_filter.h"

int main(int argc,char** argv)
{
  PCL2_Filter* filter;
  
  ros::init(argc,argv,"pcl_filter");

  filter = new PCL2_Filter(argc,argv);

  printf("PCL2_Filter initialized\n");

  filter->spin();

  delete filter;

  return 0;
}
