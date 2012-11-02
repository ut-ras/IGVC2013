
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <tabletop_object_manager/Table.h>
#ifndef _TABLETOPSEGMENTOR_H_
#define _TABLETOPSEGMENTOR_H_


namespace tabletop_object_manager 
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef pcl::KdTree<Point> KdTree;

  class TabletopSegmentor
  {
    private:
      ros::NodeHandle nh;
      ros::NodeHandle priv_nh_;
      geometry_msgs::Pose table_pose;
      PointCloud table_cloud;
      std::vector<PointCloud> clusters;
      tf::TransformListener listener;
      Table table;

      ros::Publisher cluster_pub;


      //! Min number of inliers for reliable plane detection
      int inlier_threshold;
      //! Size of downsampling grid before performing plane detection
      double plane_detection_voxel_size;
      //! Size of downsampling grid before performing clustering
      double clustering_voxel_size;
      //! Filtering of orignal point cloud along the z, y, and x axes
      double z_filter_min, z_filter_max;
      double y_filter_min, y_filter_max;
      double x_filter_min, x_filter_max;
      //! Filtering of point cloud in table frame after table detection
      double table_z_filter_min, table_z_filter_max;
      //! Min distance between two clusters
      double cluster_distance;
      //! Min number of points for a cluster
      int min_cluster_size;
      //! Clouds are transformed into this frame before processing; leave empty if clouds
      //! are to be processed in their original frame
      std::string processing_frame;

      //! Positie or negative z is closer to the "up" direction in the processing frame?
      double up_direction;
      bool flatten_table;
      //! How much the table gets padded in the horizontal direction
      double table_padding;

      // Filters
      KdTree::Ptr normals_tree,clusters_tree;
      pcl::VoxelGrid<Point> grid,grid_objects;
      pcl::PassThrough<Point> pass;
      pcl::NormalEstimation<Point, pcl::Normal> n3d;
      pcl::SACSegmentationFromNormals<Point,pcl::Normal> seg;
      pcl::ProjectInliers<Point> proj;
      pcl::ConvexHull<Point> hull;
      pcl::ExtractPolygonalPrismData<Point> prism;
      pcl::EuclideanClusterExtraction<Point> pcl_cluster;
      
    public:
      TabletopSegmentor();

      void initFilter();
      void processCloud(PointCloud::Ptr &cloud);
      geometry_msgs::Pose getPlanePose();
      Table getTable();
      std::vector<PointCloud> getClusters();

      void passThrough(PointCloud::Ptr &cloud_in,PointCloud::Ptr &cloud_out);
      void estimateNormal(PointCloud::Ptr &cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& cloud_normal);
      void planarSegmentation(PointCloud::Ptr &cloud_in,pcl::PointCloud<pcl::Normal>::Ptr & normal_ptr,PointCloud::Ptr &table_hull_ptr);
      void extractObjects(PointCloud::Ptr cloud_ptr,PointCloud::Ptr table_hull_ptr);

      bool getPlanePoints(const PointCloud& table,const tf::Transform& table_plane_trans,PointCloud::Ptr & table_points);
      Table createTable(std_msgs::Header cloud_header,const tf::Transform &table_plane_trans,const PointCloud::Ptr & table_points);

      bool convertFrame(PointCloud::Ptr &cloud_in,PointCloud::Ptr &cloud_out,std::string processing_frame);

      tf::Transform getPlaneTransform(pcl::ModelCoefficients coeffs, double up_direction,bool flatten_table);


      void getObjectClusters(const PointCloud &cloud_objects,const std::vector<pcl::PointIndices> &clusters_in,std::vector<PointCloud> &clusters_out);

  };
}

#endif
