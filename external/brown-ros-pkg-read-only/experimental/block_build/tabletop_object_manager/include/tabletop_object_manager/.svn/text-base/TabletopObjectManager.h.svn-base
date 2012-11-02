/*
   TabletopObjectManager

   Author : Jihoon Lee
   Date   : Mar 2012

   many code are adopted from tabletop_segmentation
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tabletop_object_manager/TabletopSegmentor.h>

#include <tabletop_object_manager/ObjectList.h>
#include <tabletop_object_manager/Object.h>
#include <tabletop_object_manager/Table.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>

#include <scene_filter/IsPointsVisible.h>

namespace tabletop_object_manager {
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  const std::string find_bounding_box_name = "/find_cluster_bounding_box2";
  const std::string filter_srv_name = "self_filter_srv";

  class TabletopObjectManager
  {
    private:
      ros::NodeHandle nh;
      ros::NodeHandle callback_handle;
      ros::CallbackQueue my_queue;
      ros::ServiceClient find_bounding_box_srv;
      ros::ServiceClient filter_srv;
      ros::Publisher marker_pub;
      ros::Publisher object_pub;
      ros::Publisher cloud_pub;
      ros::Subscriber cloud_sub;

      Table table;
      TabletopSegmentor segmentor;
      std::vector<Object> object_list;

    public:
      TabletopObjectManager(int argc,char** argv);

      void processCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
      void publishObjectMarkers();
        void drawTable();
        void drawObjects();
      void publishObjects();
      void spin();

      void updateTable();
      void updateObjects();
        void extractObjects();
          void extractObject(pcl::PointCloud<Point> & cloud,int i);
            void getBoxModel(PointCloud::Ptr &cloud,Object& obj);
            void getSphereModel(PointCloud::Ptr &cloud,Object& obj);
            std_msgs::ColorRGBA extractColor(PointCloud::Ptr & cloud);
        void updateExistingObjects();
          void setInvisibleObjects(std::vector<Object> &list);

  };
}
