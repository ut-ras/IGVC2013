
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
//#include "pcl/sample_consensus/method_types.h"
//#include "pcl/sample_consensus/model_types.h"
//#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/voxel_grid.h"
//#include "pcl/filters/extract_indices.h"
//#include "pcl/ModelCoefficients.h"

#include "ros/ros.h"

#ifndef _PCL2_FILTER_
#define _PCL2_FILTER_

#define SUB_TOPIC "/camera/rgb/points"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PCL2_Filter
{
  private:
    sensor_msgs::PointCloud2::Ptr new_msg;
    PointCloud::Ptr cloud_filtered;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    
    /*
    PointCloud::Ptr cloud_p;
    
    
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::SACSegmentation<pcl::PointType> seg;
    pcl::ExtractIndices<pcl::PointType> extract;
    */

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle n;
    int skip;
    double leaf_size;

  public:
    PCL2_Filter();
    PCL2_Filter(int argc,char** argv);

    ~PCL2_Filter();
    void processPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void filterCloud(PointCloud::Ptr cloud_filtered);
    void spin();
};
#endif
