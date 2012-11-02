/*
  Author : Jihoon Lee
  Date   : Feb 2012

  ObjectManager.h

*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Empty.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tabletop_segmentor/TabletopSegmentation.h>
#include <tabletop_segmentor/TabletopClusters.h>
#include <tabletop_segmentor/Table.h>
#include <object_manager/RequestTable.h>
#include <object_manager/Object.h>
#include <object_manager/ObjectList.h>
#include <object_manager/UpdateObject.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>

#ifndef _OBJECTMANAGER_H_
#define _OBJECTMANAGER_H_

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
const std::string segment_srv_name = "/tabletop_segmentation";
const std::string find_bounding_box_name = "/find_cluster_bounding_box2";
const std::string cluster_sub_name = "/tabletop_cluster";

class ObjectManager
{
  private:
    ros::NodeHandle nh;
    ros::ServiceClient segmentation_srv;
    ros::ServiceClient find_bounding_box_srv;
    ros::ServiceServer table_request_srv_server;
    ros::Publisher point_publisher;
    ros::Publisher marker_publisher;
    ros::Publisher object_publisher;
    ros::Subscriber update_object_sub;
    ros::Subscriber update_table_sub;
    ros::Subscriber cluster_sub;

    tabletop_segmentor::Table table;
    std::vector<object_manager::Object> object_list;

    unsigned int idCount;
  public:
    ObjectManager(int argc,char** argv);
    void spin();

    void drawTable(tabletop_segmentor::Table& table);

    // debug function
    void publishSegmentedPoints(std::vector<sensor_msgs::PointCloud2> clusters);

    void processCluster(const tabletop_segmentor::TabletopClusters::ConstPtr& tc); 
      void updateTable(tabletop_segmentor::Table t);

    void processScene();
      void callTabletopSegmentation(tabletop_segmentor::TabletopSegmentation& seg);
      void extractObject(std::vector<sensor_msgs::PointCloud2> clusters);
      void extractColor(std_msgs::ColorRGBA& color,sensor_msgs::PointCloud2& cloud);

    bool tableRequestCallback(object_manager::RequestTable::Request& request,object_manager::RequestTable::Response& response);

    void makeObject(object_manager::Object& obj,object_manipulation_msgs::FindClusterBoundingBox2::Response& response,std_msgs::ColorRGBA color,unsigned int i);
    void updateObjectList(std::vector<object_manager::Object> new_list);
    void publishObjectMarkers();
    void removeOldMarkers();

    void objectServerCallback(const std_msgs::Empty::ConstPtr& msg);
    void updateObjectCallback(const object_manager::Object::ConstPtr& msg);
};

#endif
