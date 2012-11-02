#include "pc2_filter.h"

// initialize node
PCL2_Filter::PCL2_Filter()
{
}

PCL2_Filter::PCL2_Filter(int argc,char** argv)
{

  sub = n.subscribe(SUB_TOPIC,5,&PCL2_Filter::processPoints,this);
  
  std::string pubtopic = SUB_TOPIC;
  
  pubtopic += "_filtered";
  pubtopic = "pclrgb";
  pub = n.advertise<PointCloud>(pubtopic,5);

  new_msg = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());
  cloud_filtered = PointCloud::Ptr(new PointCloud());
  //sor = pcl::VoxelGrid<sensor_msgs::PointCloud2>::Ptr(new pcl::VoxelGrid<sensor_msgs::PointCloud2>());

  n.param(std::string("leaf_size"), leaf_size,0.05);
  n.param(std::string("skip"), skip,10);
  
  printf("leaf size = %.2f\n",leaf_size);
  printf("skip = %d\n",skip);
  
  /*  
  cloud_p = PointCloud::Ptr(new PointCloud);
  pcl::ModelCoefficients::Ptr coefficients_t(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers_t(new pcl::PointIndices());

  coefficients = coefficients_t;
  inliers = inliers_t;

  // optional
  seg.setOptimizeCoefficients(true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);
  */
}

PCL2_Filter::~PCL2_Filter()
{
}

void PCL2_Filter::processPoints(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  skip++;
  if(skip == 1) {
    sor.setInputCloud(msg);
    sor.setLeafSize(leaf_size,leaf_size,leaf_size);
    sor.filter(*new_msg);

    pcl::fromROSMsg(*new_msg, *cloud_filtered);

    cloud_filtered->header.stamp = ros::Time::now();
    pub.publish(cloud_filtered);
  }
  else if(skip > 10)
    skip = 0;
}

/*
void PCL2_Filter::filterCloud(PointCloud::Ptr cloud_filtered)
{
  int nr_points = cloud_filtered->points.size();

  while (cloud_filtered->points.size() > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if( inliers->indices.size() == 0)
    {
      ROS_ERROR("Could not estimate a planar model for the given dataset.");
      break;
    }


    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    ROS_INFO ("PointCloud representing the planar component: %d data points.", cloud_p->width * cloud_p->height);

    extract.setNegative(true);
    extract.filter(*cloud_filtered);
  }

  cloud_filtered->header.stamp = ros::Time::now();
  pub.publish(cloud_filtered);
}
*/
void PCL2_Filter::spin()
{
  printf("in spin\n");
  ros::spin();
}
