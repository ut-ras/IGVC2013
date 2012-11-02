/*
   TabletopObjectManager

   Author : Jihoon Lee
   Date   : Mar 2012

   many code are adopted from tabletop_segmentation
 */

#include <tabletop_object_manager/TabletopObjectManager.h>

namespace tabletop_object_manager {
  TabletopObjectManager::TabletopObjectManager(int argc,char** argv)
  {
    callback_handle.setCallbackQueue(&my_queue);
    std::string topic = nh.resolveName("cloud_in");
    cloud_sub = callback_handle.subscribe(topic,1,&TabletopObjectManager::processCloud,this);


    topic = nh.resolveName("markers_out");
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(topic,10);

    topic = nh.resolveName("object_list");
    object_pub = nh.advertise<ObjectList>(topic,10);
    
    cloud_pub = nh.advertise<PointCloud>("cluster_out_debug",100);

    std::string service_name;
    // to call tabletop segmentation service
    nh.param<std::string>("filter_srv_name",service_name, filter_srv_name);
    while (!ros::service::waitForService(service_name, ros::Duration(2.0)) && nh.ok())
    {
      ROS_INFO("Waiting for %s service to come up",service_name.c_str());
    }
    if(!nh.ok()) exit(0);
    filter_srv = nh.serviceClient<scene_filter::IsPointsVisible>(service_name,true);

    nh.param<std::string>("find_bounding_srv",service_name, find_bounding_box_name);
    while (!ros::service::waitForService(service_name, ros::Duration(2.0)) && nh.ok())
    {
      ROS_INFO("Waiting for %s service to come up",service_name.c_str());
    }
    if(!nh.ok()) exit(0);
    find_bounding_box_srv = nh.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>(service_name,true);

  }

  void TabletopObjectManager::processCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    ROS_INFO("in processCloud");
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg,*cloud);

    ROS_INFO("segmentor processing");
    segmentor.processCloud(cloud);
    ROS_INFO("Done");
    updateTable();
    updateObjects();
  }

  void TabletopObjectManager::updateTable()
  {
    geometry_msgs::Pose new_table_pose = segmentor.getPlanePose();

    if(abs(table.pose.pose.position.z - new_table_pose.position.z) >= 0.05 || table.pose.header.seq == 0)
    {
      ROS_INFO("table updated");
      table = segmentor.getTable();
    }
  }

  void TabletopObjectManager::updateObjects()
  {
    if(object_list.size() == 0)
    {
      extractObjects();
    }
    else 
    {
      updateExistingObject();
      extractObjects();
    }
  }

  void TabletopObjectManager::extractObjects()
  {
    std::vector<PointCloud> clusters = segmentor.getClusters();
    object_list.resize(clusters.size());
  
    for(unsigned int i = 0 ; i < clusters.size(); i++)
    {
      ROS_INFO("Extract object %d",i);
      extractObject(clusters[i],i);
      ROS_INFO("Done");
    }
  }

  void TabletopObjectManager::extractObject(PointCloud& cloud,int i)
  {
    Object obj;
    PointCloud::Ptr cloud_ptr(new PointCloud(cloud));

//    cloud_pub.publish(cloud);
    //getSphereModel(cloud_ptr,obj);
    getBoxModel(cloud_ptr,obj);
    
    obj.id = i;
    object_list[i] = obj;     
  }

  void TabletopObjectManager::getBoxModel(PointCloud::Ptr &cloud,Object& obj)
  {
    object_manipulation_msgs::FindClusterBoundingBox2 fbb;
    
    if(!find_bounding_box_srv.call(fbb))
    {
      ROS_ERROR("Call to find_bounding_box service failed");
    }

    if(fbb.response.error_code != fbb.response.SUCCESS)
    {
      ROS_ERROR("Find Bounding Box failed");
      return;
    }  

    obj.header = cloud->header;                        
    obj.pose = fbb.response.pose.pose;
    obj.dim = fbb.response.box_dims;
    obj.color = extractColor(cloud);
    obj.type =  obj.BOX;
    obj.status = Object::VISIBLE;
  }

  void TabletopObjectManager::getSphereModel(PointCloud::Ptr &cloud,Object& obj)
  {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr sphere_inliers_ptr(new pcl::PointIndices);
    KdTree::Ptr normals_tree(new pcl::KdTreeFLANN<Point>());
    pcl::NormalEstimation<Point, pcl::Normal> n3d;
    pcl::SACSegmentationFromNormals<Point,pcl::Normal> seg;
    // Normal estimation parameters
    n3d.setKSearch(10);
    n3d.setSearchMethod(normals_tree);
    // Table model fitting parameters
    seg.setDistanceThreshold(0.005);
    seg.setMaxIterations(1000);
//    seg.setNormalDistanceWeight(0.1);
    seg.setRadiusLimits(0.024,0.025);
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_LMEDS);
    seg.setProbability(0.99);

    n3d.setInputCloud(cloud);
    n3d.compute(*cloud_normal_ptr);

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normal_ptr);
    seg.segment(*sphere_inliers_ptr,*coefficients_sphere);
    ROS_INFO("cloud  size = %d",(int)cloud->points.size());
    ROS_INFO("inlier size = %d",(int)sphere_inliers_ptr->indices.size());

    obj.header = cloud->header;                        
    obj.pose.position.x = coefficients_sphere->values[0];
    obj.pose.position.y = coefficients_sphere->values[1];
    obj.pose.position.z = coefficients_sphere->values[2];
    obj.dim.x = coefficients_sphere->values[3] * 2;
    obj.dim.y = coefficients_sphere->values[3] * 2;
    obj.dim.z = coefficients_sphere->values[3] * 2;
    obj.color = extractColor(cloud);
    obj.type =  obj.SPHERE;
    obj.status = Object::VISIBLE;
  }

  std_msgs::ColorRGBA TabletopObjectManager::extractColor(PointCloud::Ptr & cloud)
  {
    std_msgs::ColorRGBA color;
    unsigned int rgb = 0;
    unsigned int r,g,b,size;
    unsigned int i;

    r = g = b =0;
    size = cloud->points.size() * 255;
  
    for(i =0; i < cloud->points.size(); i++)
    {
      rgb = *reinterpret_cast<int*>(&(cloud->points[i].rgb));
  
      r += ((rgb >> 16) & 0xff);
      g += ((rgb >> 8) & 0xff);
      b += ((rgb) & 0xff);
    }
  
    color.r = (float) r/ (float)size;
    color.g = (float) g/ (float)size;
    color.b = (float) b/ (float)size;
    color.a = 1.0f;
    return color;
  }

  void TabletopObjectManager::updateExistingObjects()
  {
    std::vector<Object> new_list;

    setInvisibleObjects(new_list);
    updateObjectList(new_list);

    for(i = 0 ; i <new_list.size(); i ++)
      object_list.push_back(new_list[i]);
  }

  void TabletopObjectManager::setInvisibleObjects(std::vector<Object> &new_list)
  {
    unsigned int i;
    scene_filter::IsPointsVisible ipv;

    for(i=0; i < object_list.size();i ++)
    {
      ipv.request.point = object_list[i].pose.position;

      if(!filter_srv.call(ipv))
      {
        ROS_ERROR("Call to filter_srv failed");
      }

      object_list[i].status = (ipv.response.result == ipv.response.SHADOW)?(Object::INSHADOW):(Object::VISIBLE);
      new_list.push_back(object_list[i]);
    }
  }

  void TabletopObjectManager::updateObjectList(std::vector<Object> &new_list)
  {
    unsigned int i,j;
    float dist;
    bool updated;
   
    for(i = 0; i < object_list.size(); i++)
    {
      updated = false;

      for(j = 0; j < new_list.size(); j++)
      {
        dist = 0;
        dist += fabs(new_list[j].pose.position.x - object_list[i].pose.position.x);
        dist += fabs(new_list[j].pose.position.y - object_list[i].pose.position.y);

        if(dist < 0. 1)
        {
          updated = true;
        }

      }



  }

  void TabletopObjectManager::spin()
  {
    ros::AsyncSpinner spinner(0, &my_queue);
    spinner.start();
    ROS_INFO("callback thread started");


    while(ros::ok())
    {
      publishObjects();
      publishObjectMarkers();
//      ros::spinOnce();
      ros::Duration(0.05f).sleep();
    }
    spinner.stop();
  }

  void TabletopObjectManager::publishObjects()
  {

  }


  void TabletopObjectManager::publishObjectMarkers()
  {
    drawTable();
    drawObjects();
  }
  
  void TabletopObjectManager::drawTable()
  {
    visualization_msgs::MarkerArray marray;
    marray.markers.resize(1);
    marray.markers[0].header = table.pose.header;
    marray.markers[0].header.stamp = ros::Time::now();

    marray.markers[0].ns = "table";
    marray.markers[0].id =  99;
    marray.markers[0].type = visualization_msgs::Marker::TRIANGLE_LIST;
    marray.markers[0].action = visualization_msgs::Marker::ADD;;
    marray.markers[0].pose = table.pose.pose;
    marray.markers[0].points.resize(6);
    marray.markers[0].scale.x = 1;
    marray.markers[0].scale.y = 1;
    marray.markers[0].scale.z = 1;
    marray.markers[0].color.r = 0.5;
    marray.markers[0].color.g = 0.5;
    marray.markers[0].color.b = 0.5;
    marray.markers[0].color.a = 0.5;

    marray.markers[0].points[0].x = table.x_min;
    marray.markers[0].points[0].y = table.y_min;
    marray.markers[0].points[0].z = 0.0f;
    marray.markers[0].points[1].x = table.x_min;
    marray.markers[0].points[1].y = table.y_max;
    marray.markers[0].points[1].z = 0.0f;
    marray.markers[0].points[2].x = table.x_max;
    marray.markers[0].points[2].y = table.y_max;
    marray.markers[0].points[2].z = 0.0f;
  
    marray.markers[0].points[3].x = table.x_max;
    marray.markers[0].points[3].y = table.y_max;
    marray.markers[0].points[3].z = 0.0f;         
    marray.markers[0].points[4].x = table.x_max;
    marray.markers[0].points[4].y = table.y_min;
    marray.markers[0].points[4].z = 0.0f;         
    marray.markers[0].points[5].x = table.x_min;
    marray.markers[0].points[5].y = table.y_min;
    marray.markers[0].points[5].z = 0.0f;         
  
    marker_pub.publish(marray);
  }

  void TabletopObjectManager::drawObjects()
  {
    unsigned int i;
    visualization_msgs::MarkerArray marray;

    marray.markers.resize(object_list.size());
    for(i = 0 ; i < object_list.size(); i++)
    {
      visualization_msgs::Marker m;
      m.header = object_list[i].header;
      m.header.stamp = ros::Time::now();
      m.id = i;
      m.action = visualization_msgs::Marker::ADD;
      m.lifetime = ros::Duration(3.0f);

      switch(object_list[i].type)
      {
        case Object::SPHERE:
          m.type = visualization_msgs::Marker::SPHERE;
          m.pose = object_list[i].pose;
          m.scale = object_list[i].dim;
          m.color = object_list[i].color;
          break;
        case Object::BOX:
          break;
        default:
          break;
      }

      marray.markers.push_back(m);
    }
    marker_pub.publish(marray);
  }

}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"tabletop_object_manager");
  tabletop_object_manager::TabletopObjectManager tom(argc,argv);

  ROS_INFO("initialized");
  tom.spin();

  ROS_INFO("Done");
  return 0;
}
