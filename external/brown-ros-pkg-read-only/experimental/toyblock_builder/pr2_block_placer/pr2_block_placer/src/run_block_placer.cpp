
#include <pr2_block_placer/block_placer.h>

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"block_placer");
  ros::NodeHandle nh;

  BlockPlacer *bp = new BlockPlacer(argc,argv);

  ROS_INFO("Block Placer initialized");
  bp->spin();

  delete bp;

  return 0;
}
