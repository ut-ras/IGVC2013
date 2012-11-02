
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
  
// Author(s): Marius Muja and Matei Ciocarlie

/*
   Modified by Jihoon Lee , Brown University
   Date : Feb 2012

   - removed pointcloud2 to pointcloud conversion
   - republish colored pointcloud
 */

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tabletop_object_detector/marker_generator.h>
#include <tabletop_segmentor/TabletopSegmentation.h>
#include <tabletop_segmentor/TabletopClusters.h>

namespace tabletop_segmentor {

const std::string CLUSTER_PUB_NAME = "tabletop_cluster";

class TabletopSegmentor 
{
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher marker_pub_;
  //! Service server for object detection
  ros::ServiceServer segmentation_srv_;

  ros::Publisher cluster_pub;
  ros::Subscriber cloud_sub;


  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! Min number of inliers for reliable plane detection
  int inlier_threshold_;
  //! Size of downsampling grid before performing plane detection
  double plane_detection_voxel_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
  //! Filtering of original point cloud along the z, y, and x axes
  double z_filter_min_, z_filter_max_;
  double y_filter_min_, y_filter_max_;
  double x_filter_min_, x_filter_max_;
  //! Filtering of point cloud in table frame after table detection
  double table_z_filter_min_, table_z_filter_max_;
  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;
  //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;
  bool flatten_table_;
  //! How much the table gets padded in the horizontal direction
  double table_padding_;

  //! A tf transform listener
  tf::TransformListener listener_;

  //------------------ Individual processing steps -------

  //! Converts raw table detection results into a Table message type
  template <class PointCloudType>
  Table getTable(std_msgs::Header cloud_header, const tf::Transform &table_plane_trans,
		 const PointCloudType &table_points);

  template <typename PointT> 
  bool getPlanePoints (const pcl::PointCloud<PointT> &table,const tf::Transform& table_plane_trans,sensor_msgs::PointCloud &table_points);

  template <typename PointT> 
  void getClusters (const pcl::PointCloud<PointT> &cloud_objects,const std::vector<pcl::PointIndices> &clusters_in, std::vector<sensor_msgs::PointCloud2> &clusters_out);

  template <class PointCloudType>
  void straightenPoints(PointCloudType &points, const tf::Transform& table_plane_trans, const tf::Transform& table_plane_trans_flat);

  tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction, bool flatten_plane);


  //! Converts table convex hull into a triangle mesh to add to a Table message
  //template <class PointCloudType>
  //void addConvexHullTable(Table &table, const PointCloudType &convex_hull, bool flatten_table);

  //! Publishes rviz markers for the given tabletop clusters
  template <class PointCloudType>
  void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);
  
  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  void processCloud(pcl::PointCloud<Point>::Ptr& cloud,TabletopSegmentation::Response &response);
  
  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);

public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TabletopSegmentor(ros::NodeHandle nh);
  ~TabletopSegmentor();

  //------------------ Callbacks -------------------
  void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr cloud_in);
    bool convertFrame(pcl::PointCloud<Point>::Ptr cloud_in,pcl::PointCloud<Point>::Ptr cloud_out);

  //! Callback for service calls
  bool serviceCallback(TabletopSegmentation::Request &request, TabletopSegmentation::Response &response);

};
}

