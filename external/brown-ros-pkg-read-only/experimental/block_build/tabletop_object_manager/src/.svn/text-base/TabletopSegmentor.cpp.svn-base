
#include <tabletop_object_manager/TabletopSegmentor.h>

namespace tabletop_object_manager
{
  TabletopSegmentor::TabletopSegmentor()
    : priv_nh_("~")
  {
    //initialize operational flags
    priv_nh_.param<int>("inlier_threshold", inlier_threshold, 300);
    priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size, 0.01);
    priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size, 0.003);
    priv_nh_.param<double>("z_filter_min", z_filter_min, 0.4);
    priv_nh_.param<double>("z_filter_max", z_filter_max, 1.25);
    priv_nh_.param<double>("y_filter_min", y_filter_min, -1.0);
    priv_nh_.param<double>("y_filter_max", y_filter_max, 1.0);
    priv_nh_.param<double>("x_filter_min", x_filter_min, -1.0);
    priv_nh_.param<double>("x_filter_max", x_filter_max, 1.0);
    priv_nh_.param<double>("table_z_filter_min", table_z_filter_min, 0.01);
    priv_nh_.param<double>("table_z_filter_max", table_z_filter_max, 0.50);
    priv_nh_.param<double>("cluster_distance", cluster_distance, 0.03);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size, 300);
    priv_nh_.param<std::string>("processing_frame", processing_frame, "");
    priv_nh_.param<double>("up_direction", up_direction, -1.0);   
    priv_nh_.param<bool>("flatten_table", flatten_table, false);   
    priv_nh_.param<double>("table_padding", table_padding, 0.0);   
    if(flatten_table) ROS_DEBUG("flatten_table is true");
    else ROS_DEBUG("flatten_table is false");

    initFilter();

    cluster_pub = nh.advertise<PointCloud>("cluster_out_debug",100);

  }

  void TabletopSegmentor::initFilter()
  {
    // filtering parameters
    grid.setLeafSize(plane_detection_voxel_size,plane_detection_voxel_size,plane_detection_voxel_size);
    grid_objects.setLeafSize(clustering_voxel_size,clustering_voxel_size,clustering_voxel_size);
    grid.setFilterFieldName("z");
    grid.setFilterLimits(z_filter_min,z_filter_max);
    grid.setDownsampleAllData(true);
    grid_objects.setDownsampleAllData(true);

    normals_tree = boost::make_shared<pcl::KdTreeFLANN<Point> >();
    clusters_tree = boost::make_shared<pcl::KdTreeFLANN<Point> >();

    // Normal estimation parameters
    n3d.setKSearch(10);
    n3d.setSearchMethod(normals_tree);
    // Table model fitting parameters
    seg.setDistanceThreshold(0.05);
    seg.setMaxIterations(1000);
    seg.setNormalDistanceWeight(0.1);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setProbability(0.99);

    proj.setModelType(pcl::SACMODEL_PLANE);

    // Clustering parameters
    pcl_cluster.setClusterTolerance(cluster_distance);
    pcl_cluster.setMinClusterSize(min_cluster_size);
    pcl_cluster.setSearchMethod(clusters_tree);
  }

  geometry_msgs::Pose TabletopSegmentor::getPlanePose()
  {
    return table_pose;
  }

  Table TabletopSegmentor::getTable()
  {
    return table;
  }

  std::vector<PointCloud> TabletopSegmentor::getClusters()
  {
    return clusters;
  }


  void TabletopSegmentor::processCloud(PointCloud::Ptr &cloud)
  {
    PointCloud::Ptr converted_cloud(new PointCloud);
    PointCloud::Ptr converted_cloud2(new PointCloud);
    convertFrame(cloud,converted_cloud,processing_frame);
    if(converted_cloud->points.size() == 0)
      return;


    // Step1 : Filter, remove NaNs and downsample
    passThrough(converted_cloud,converted_cloud2);
    //cluster_pub.publish(converted_cloud2);


    // Step2 : Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_ptr(new pcl::PointCloud<pcl::Normal>);
    estimateNormal(converted_cloud2,cloud_normal_ptr);


    // Step3 : extract Table
    PointCloud::Ptr table_hull_ptr(new PointCloud);
    planarSegmentation(converted_cloud2,cloud_normal_ptr,table_hull_ptr);

    // Step4 : segment out the object clusters
    extractObjects(converted_cloud2,table_hull_ptr);
  }

  bool TabletopSegmentor::convertFrame(PointCloud::Ptr & cloud_in,PointCloud::Ptr & cloud_out,std::string processing_frame)
  {
    if(!processing_frame.empty())
    {
      while(1) {
        int current_try = 0, max_tries = 10;
        bool transform_success = true;

        try 
        {
          pcl_ros::transformPointCloud(processing_frame,*cloud_in,*cloud_out,listener);
        }
        catch(tf::TransformException ex)
        {
          transform_success = false;
          if(++current_try >= max_tries)
          {
            ROS_ERROR("Failed to transform cloud from frame %s into frame %s in %d attempt(s)",cloud_in->header.frame_id.c_str(),processing_frame.c_str(),current_try);
            return false;
          }
          ROS_DEBUG("Failed to transform cloud from frame %s into frame %s in %d attempt(s)",cloud_in->header.frame_id.c_str(),processing_frame.c_str(),current_try);
          ros::Duration(0.1).sleep();
        }
        if(transform_success) break;
      }
    }
    else {
      cloud_out = cloud_in;
    }
    return true;
  }

  void TabletopSegmentor::passThrough(PointCloud::Ptr &cloud_in,PointCloud::Ptr &cloud_out)
  {
    pcl::PassThrough<Point> pass;

    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_filter_min,z_filter_max);
    PointCloud::Ptr z_cloud_filtred_ptr(new PointCloud);
    pass.filter(*z_cloud_filtred_ptr);

    pass.setInputCloud(z_cloud_filtred_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_filter_min,y_filter_max);
    PointCloud::Ptr y_cloud_filtered_ptr(new PointCloud);
    pass.filter(*y_cloud_filtered_ptr);

    pass.setInputCloud(y_cloud_filtered_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_filter_min,x_filter_max);
    PointCloud::Ptr cloud_filtered_ptr(new PointCloud);
    pass.filter(*cloud_out);
    
//    grid.setInputCloud(cloud_filtered_ptr);
//    grid.filter(*cloud_out);
  }

  void TabletopSegmentor::estimateNormal(PointCloud::Ptr &cloud_in,pcl::PointCloud<pcl::Normal>::Ptr &cloud_out)
  {
    n3d.setInputCloud(cloud_in);
    n3d.compute(*cloud_out);
  }

  void TabletopSegmentor::planarSegmentation(PointCloud::Ptr &cloud_in,pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal,PointCloud::Ptr & table_hull_ptr)
  {
    pcl::PointIndices::Ptr table_inliers_ptr(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr table_coefficients_ptr(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud_in);
    seg.setInputNormals(cloud_normal);
    seg.segment(*table_inliers_ptr,*table_coefficients_ptr);

    PointCloud::Ptr table_projected_ptr(new PointCloud);
    proj.setInputCloud(cloud_in);
    proj.setIndices(table_inliers_ptr);
    proj.setModelCoefficients(table_coefficients_ptr);
    proj.filter(*table_projected_ptr);

    tf::Transform table_plane_trans = getPlaneTransform(*table_coefficients_ptr,up_direction, false);
    tf::Transform table_plane_trans_flat;

    PointCloud::Ptr table_hull_points(new PointCloud);
    hull.setInputCloud(table_projected_ptr);
    hull.reconstruct(*table_hull_ptr);

    if(!flatten_table)
    {
      // --- [ Take the points projected on the table and transform them into the PointCloud message
      //  while also transforming them into the table's coordinate system
      getPlanePoints (*table_hull_ptr, table_plane_trans,table_hull_points);
    }
    else
    {
      // if flattening the table, find the center of the convex hull and move the table frame there
      table_plane_trans_flat = getPlaneTransform(*table_coefficients_ptr,up_direction,flatten_table);
      tf::Vector3 flat_table_pos(0,0,0);
      double avg_x,avg_y,avg_z;
      avg_x = avg_y = avg_z = 0;

      for(size_t i = 0; i< table_projected_ptr->points.size() ; i++)
      {
        avg_x += table_projected_ptr->points[i].x;
        avg_y += table_projected_ptr->points[i].y;
        avg_z += table_projected_ptr->points[i].z;
      }
      avg_x /= table_projected_ptr->points.size();
      avg_y /= table_projected_ptr->points.size();
      avg_z /= table_projected_ptr->points.size();

      // place the new table frame in the center of the convex hull
      flat_table_pos[0] = avg_x;
      flat_table_pos[1] = avg_y;
      flat_table_pos[2] = avg_z;
      table_plane_trans_flat.setOrigin(flat_table_pos);

      // shift the non-flat table frame to the centoer of the convex hull as well
      table_plane_trans.setOrigin(flat_table_pos);

      // --- [ Take the points projected on the table and transform them into the PointCloud message
      //  while also transforming them into the flat table's coordinate system
      PointCloud::Ptr flat_table_points(new PointCloud);
      getPlanePoints(*table_projected_ptr,table_plane_trans_flat,flat_table_points);

      // --- [ Convert the convex hull points to flat table frame
      table = createTable(cloud_in->header,table_plane_trans_flat,flat_table_points); 
      table_cloud = *table_projected_ptr;
      //cluster_pub.publish(*table_projected_ptr);

      // ---[ Convert the convex hull points to flat table frame 
      getPlanePoints(*table_hull_ptr,table_plane_trans_flat,table_hull_points);
    }
  }

  bool TabletopSegmentor::getPlanePoints(const PointCloud& table,const tf::Transform& table_plane_trans,PointCloud::Ptr & table_points) 
  {
    // prepare the output
    table_points->header = table.header;
    table_points->points.resize(table.points.size());
    for(size_t i = 0; i <table.points.size(); i++)
    {
      table_points->points[i].x = table.points[i].x;
      table_points->points[i].y = table.points[i].y;
      table_points->points[i].z = table.points[i].z;
    }

    // Transform the data
    tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp, table.header.frame_id, "table_frame");
    listener.setTransform(table_pose_frame);
    std::string error_msg;
    if(!listener.canTransform("table_frame",table_points->header.frame_id, table_points->header.stamp, &error_msg))
    {
      ROS_ERROR("Can not transform point cloud from %s to table frame; error %s", table_points->header.frame_id.c_str(), error_msg.c_str());
      return false;
    }
    
    std::string frame = "table_frame";
    convertFrame(table_points,table_points,frame);
    table_points->header.stamp = table.header.stamp;
    table_points->header.frame_id = "table_frame";
    return true;
  }

  Table TabletopSegmentor::createTable(std_msgs::Header cloud_header,const tf::Transform &table_plane_trans,const PointCloud::Ptr & table_points)
  {
    Table table;

    // get the extents of the table
    if(!table_points->points.empty())
    {
      table.x_min = table_points->points[0].x;
      table.x_max = table_points->points[0].x;
      table.y_min = table_points->points[0].y;
      table.y_max = table_points->points[0].y;
    }
    
    for(size_t i=1; i < table_points->points.size(); i++)
    {
      if(table_points->points[i].x < table.x_min && table_points->points[i].x >-3.0) table.x_min = table_points->points[i].x;
      if(table_points->points[i].x > table.x_max && table_points->points[i].x < 3.0) table.x_max = table_points->points[i].x;
      if(table_points->points[i].y < table.y_min && table_points->points[i].y >-3.0) table.y_min = table_points->points[i].y;
      if(table_points->points[i].y > table.y_max && table_points->points[i].y < 3.0) table.y_max = table_points->points[i].y;
    }

    geometry_msgs::Pose table_pose;
    tf::poseTFToMsg(table_plane_trans,table_pose);
    table.pose.pose = table_pose;
    table.pose.header = cloud_header;
    
    return table;
  }

  // Assume plane coefficients are of  the form ax+by+cz+d=0, normalized */
  tf::Transform TabletopSegmentor::getPlaneTransform(pcl::ModelCoefficients coeffs, double up_direction,bool flatten_plane)
  {
    ROS_ASSERT(coeffs.values.size() > 3);
    double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
    // assume plane coefficients are normalized
    btVector3 position(-a*d, -b*d, -c*d);
    btVector3 z(a,b,c);

    // if we are flattening the plane, make z just be (0,0,up_direction)
    if(flatten_plane)
    {
      z[0] = z[1] = 0;
      z[2] = up_direction;
    }
    else 
    {
      if (z.dot(btVector3(0,0,up_direction))<0)
      {
        z = -1.0 * z;
      }
    }

    btVector3 x(1, 0, 0);
    if( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1 ,0);
    btVector3 y = z.cross(x).normalized();
    x = y.cross(z).normalized();

    btMatrix3x3 rotation;
    rotation[0] =x;
    rotation[1] =y;
    rotation[2] =z;
    rotation = rotation.transpose();
    btQuaternion orientation;
    rotation.getRotation(orientation);
    return tf::Transform(orientation, position);
  }

  void TabletopSegmentor::extractObjects(PointCloud::Ptr cloud_ptr,PointCloud::Ptr table_hull_ptr)
  {

    pcl::ExtractPolygonalPrismData<Point> prism;
    pcl::PointIndices cloud_object_indices;
    prism.setInputCloud(cloud_ptr);
    prism.setInputPlanarHull(table_hull_ptr);
    prism.setHeightLimits(table_z_filter_min,table_z_filter_max);
    prism.segment(cloud_object_indices);

    pcl::PointCloud<Point>::Ptr cloud_objects_ptr(new PointCloud);
    pcl::ExtractIndices<Point> extract_object_indices;
    extract_object_indices.setInputCloud(cloud_ptr);
    extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
    extract_object_indices.filter (*cloud_objects_ptr);

    if (cloud_objects_ptr->points.empty())
    {
      ROS_INFO("No objects on table");
      return;
    }
    cluster_pub.publish(*cloud_objects_ptr);

    std::vector<pcl::PointIndices> clusters2;

    pcl_cluster.setInputCloud (cloud_objects_ptr);
    pcl_cluster.extract (clusters2);

    // ---[ convert clusters into the pointcloud message
    getObjectClusters (*cloud_objects_ptr, clusters2, clusters);
  }

  void TabletopSegmentor::getObjectClusters(const PointCloud &cloud_objects,const std::vector<pcl::PointIndices> &clusters_in,std::vector<PointCloud> &clusters_out)
  {
    ros::Time t = ros::Time::now();
    clusters_out.clear();
  
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters_in.begin (); it != clusters_in.end (); ++it)
    {
      PointCloud cloud_cluster;
      
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster.points.push_back (cloud_objects.points[*pit]); //*
      cloud_cluster.width = cloud_cluster.points.size ();
      cloud_cluster.height = 1;
      cloud_cluster.header = cloud_objects.header;
      cloud_cluster.header.stamp = t;
      cloud_cluster.is_dense = true;
  
      clusters_out.push_back(cloud_cluster);
    }
  }
}

