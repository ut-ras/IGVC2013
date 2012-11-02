/*
  Author  : Jihoon Lee
  Date      : Feb. 2012

  ObjectManager.cpp
 */

#include<object_manager/ObjectManager.h>

ObjectManager::ObjectManager(int argc,char** argv)
{
  idCount = 0;
  std::string service_name;

  // to call tabletop segmentation service
  nh.param<std::string>("segmentation_srv",service_name, segment_srv_name);
  while (!ros::service::waitForService(service_name, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for %s service to come up",service_name.c_str());
  }
  if(!nh.ok()) exit(0);
  segmentation_srv =nh.serviceClient<tabletop_segmentor::TabletopSegmentation>(service_name,true);

  // to call find_cluster bounding box service
  nh.param<std::string>("find_bounding_srv",service_name, find_bounding_box_name);
  while (!ros::service::waitForService(service_name, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for %s service to come up",service_name.c_str());
  }
  if(!nh.ok()) exit(0);
  find_bounding_box_srv = nh.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>(service_name,true);


  // To process a request from block_build_manager
  nh.param<std::string>("update_table_objects_topic",service_name, "/blocknlp/update_table");
  update_table_sub = nh.subscribe(service_name,5,&ObjectManager::objectServerCallback,this);

  // To process a table request. 
  nh.param<std::string>("request_table_srv",service_name, "/blocknlp/request_table");
  table_request_srv_server = nh.advertiseService<object_manager::RequestTable::Request,object_manager::RequestTable::Response>(service_name, boost::bind(&ObjectManager::tableRequestCallback,this,_1,_2));

  // To update an object in hand
  nh.param<std::string>("update_object_topic",service_name, "/blocknlp/update_object");
  update_object_sub = nh.subscribe(service_name,5,&ObjectManager::updateObjectCallback,this);


  nh.param<std::string>("tabletop_cluster_topic",service_name,cluster_sub_name );
  cluster_sub = nh.subscribe(service_name,5,&ObjectManager::processCluster,this);

  // publish marker information
  marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("boundingmarkers",100);

  // publish object information
  object_publisher = nh.advertise<object_manager::ObjectList>("/blocknlp/object_list",100);

  // debugging purpose
  point_publisher = nh.advertise<PointCloud>("segmented_points",100);
}

void ObjectManager::callTabletopSegmentation(tabletop_segmentor::TabletopSegmentation& segmentation)
{
  //ROS_INFO("Calling Tabletop segmentation service");
  if(!segmentation_srv.call(segmentation))
  {
    ROS_ERROR("Call to segmentation service failed");
  }

  if(segmentation.response.result != segmentation.response.SUCCESS)
  {
    ROS_ERROR("Segmentation failed");
    return;
  }  
  //ROS_INFO("Segmentation service succeeded. Detected %d clsusters",(int)segmentation.response.clusters.size());
  
  return;
}

void ObjectManager::extractObject(std::vector<sensor_msgs::PointCloud2> clusters)
{
  object_manipulation_msgs::FindClusterBoundingBox2 fbb;
  object_manager::Object obj;
  std::vector<object_manager::Object> new_obj_list;
  std_msgs::ColorRGBA color;

  for(unsigned int i =0; i < clusters.size(); i++)
  {
    fbb.request.cluster = clusters[i];
  
    if(!find_bounding_box_srv.call(fbb))
    {
      ROS_ERROR("Call to find_bounding_box service failed");
    }

    if(fbb.response.error_code != fbb.response.SUCCESS)
    {
      ROS_ERROR("Find Bounding Box failed");
      return;
    }  
//    ROS_INFO("Find Bounding Box succeeded.");
    
    extractColor(color,clusters[i]);
    makeObject(obj,fbb.response,color,idCount++);
    new_obj_list.push_back(obj);

    // add into object list
  }

  removeOldMarkers();
  updateObjectList(new_obj_list);
}

void ObjectManager::extractColor(std_msgs::ColorRGBA& color,sensor_msgs::PointCloud2& p_cloud)
{
  unsigned int i;
  unsigned int rgb = 0;
//  unsigned int max_val;
//  unsigned int c;
  unsigned int r,g,b,size;

  r = g = b =0;

  std::map<unsigned int,unsigned int> histogram;
  std::map<unsigned int,unsigned int>::iterator it;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(p_cloud,cloud);

  size = cloud.size() * 255;

  for(i =0; i < cloud.points.size(); i++)
  {
    rgb = *reinterpret_cast<int*>(&(cloud.points[i].rgb));

    r += ((rgb >> 16) & 0xff);
    g += ((rgb >> 8) & 0xff);
    b += ((rgb) & 0xff);
  }

  color.r = (float) r/ (float)size;
  color.g = (float) g/ (float)size;
  color.b = (float) b/ (float)size;

  /*
  color.g = 0.0;
  if(r > b)
  {
    color.r = 1.0;
    color.b = 0.0;
  }
  else {
    color.b = 1.0;
    color.r = 0.0;
  }*/

  /*
  // extract rgba from 32 bit
  // red
  c = ((rgb >> 16) & 0xff);
  color.r = (float)((float)c / 255);

  // green
  c = ((rgb >> 8) & 0xff);
  color.g = (float)((float)c / 255);

   // blue
   c = ((rgb) & 0xff);           
   color.b = (float)((float)c / 255);

   color.g = 0;
   if(color.r > color.b)
   {
     color.r = 1.0f;
     color.b = 0.0f;
   }
   else {
     color.r = 0.0f;
     color.b = 1.0f;
   }
*/

   // alpha
   color.a = 1.0f;
}

void ObjectManager::removeOldMarkers()
{
  visualization_msgs::MarkerArray marray;
  for(unsigned int i =0; i < object_list.size(); i++)
  {
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = object_list[i].header.frame_id;
    m.id = object_list[i].id;
    m.action = visualization_msgs::Marker::DELETE;
    m.ns = "bounding_boxes";
    marray.markers.push_back(m);
  }
  marker_publisher.publish(marray);

}

void ObjectManager::updateObjectList(std::vector<object_manager::Object> new_list)
{
  unsigned int i,j;
  //unsigned int id;
  std_msgs::ColorRGBA color;
  bool updated;
  float dist;

  if(object_list.size() == 0) {
    object_list.insert(object_list.begin(),new_list.begin(),new_list.end());
    return;
  }

  for(i = 0;i <object_list.size(); i++)
  {
    updated = false;
//    printf("Id = %d\n",object_list[i].id);
    for(j =0; j < new_list.size(); j++) 
    {
      dist =0;
      dist += fabs(new_list[j].pose.position.x - object_list[i].pose.position.x);
      dist += fabs(new_list[j].pose.position.y - object_list[i].pose.position.y);
//      dist += fabs(new_list[j].pose.position.z - object_list[i].pose.position.z);

      if(dist < 0.1) {
//        id = object_list[i].id;
//        color = object_list[i].color;
//        object_list[i] = new_list[j];
//        object_list[i].id = id;
//        object_list[i].color;
        updated = true;
        new_list.erase(new_list.begin() + j);

//        printf("Old ID = %d New ID = %d  DIST = %f\n",object_list[i].id,new_list[j].id,dist);
        break;
      }
    }

    // if obj is not inhand and not updated. remove from object list
    if(updated == false && object_list[i].inhand == false)
    {
//      printf("ID = %d is erased\n",object_list[i].id);
      object_list.erase(object_list.begin() + i);
      i--;
    }
  }

  // add new objects
  for(i =0; i < new_list.size();i ++)
  {
//    printf("ID = %d is added\n",new_list[i].id);
    object_list.push_back(new_list[i]);
  }

//  std::cout << "=======================" << std::endl;
}

void ObjectManager::publishObjectMarkers()
{
  
  visualization_msgs::MarkerArray marray;
  marray.markers.resize(object_list.size());

  for(unsigned int i =0; i < object_list.size(); i++)
  {
    marray.markers[i].header = object_list[i].header;
    marray.markers[i].header.stamp = ros::Time::now();
    marray.markers[i].ns = "bounding_boxes";
    marray.markers[i].id = object_list[i].id;
    marray.markers[i].type = visualization_msgs::Marker::CUBE;
    marray.markers[i].action = visualization_msgs::Marker::ADD;
    marray.markers[i].pose = object_list[i].pose;
    marray.markers[i].scale = object_list[i].dim;
    marray.markers[i].color = object_list[i].color;

    /*
    marray.markers[i].color.r = object_list[i].dim.x * 10;
    marray.markers[i].color.g = object_list[i].dim.y * 10;
    marray.markers[i].color.b = object_list[i].dim.z * 10;
    marray.markers[i].color.a = 1.0f;*/
  }

  marker_publisher.publish(marray);
//  ROS_INFO("Object Published");
   
}

void ObjectManager::makeObject(object_manager::Object& obj,object_manipulation_msgs::FindClusterBoundingBox2::Response& response,std_msgs::ColorRGBA color,unsigned int i)
{
  obj.header = response.pose.header;
  obj.header.stamp = ros::Time::now();

  obj.id     = i; 
  obj.pose   = response.pose.pose; 
  obj.dim    = response.box_dims;

  obj.color = color;
    
  //obj.color.r = obj.dim.x * 10;
  //obj.color.g = obj.dim.y * 10;
  //obj.color.b = obj.dim.z * 10;
  //obj.color.a = 1.0f;

  obj.inhand = false;

}


void ObjectManager::drawTable(tabletop_segmentor::Table& table)
{
  visualization_msgs::MarkerArray marray;
  marray.markers.resize(1);
  marray.markers[0].header = table.pose.header;
  marray.markers[0].header.stamp = ros::Time::now();

  marray.markers[0].ns = "table";
  marray.markers[0].id = 99;
  marray.markers[0].type = visualization_msgs::Marker::TRIANGLE_LIST;
  marray.markers[0].action = visualization_msgs::Marker::ADD;
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


  marker_publisher.publish(marray);
}

void ObjectManager::processCluster(const tabletop_segmentor::TabletopClusters::ConstPtr& tabletop)
{
  updateTable(tabletop->table);
  extractObject(tabletop->clusters);
}

void ObjectManager::updateTable(tabletop_segmentor::Table t)
{

  if(abs(t.pose.pose.position.z - table.pose.pose.position.z) >= 0.05)
    table = t;
}

void ObjectManager::processScene()
{
  // get point cloud cluster
  tabletop_segmentor::TabletopSegmentation segmentation;

  // call tabletop segmentation service
  callTabletopSegmentation(segmentation);

  // setting table
  table = segmentation.response.table;

  //publishSegmentedPoints(segmentation.response.clusters);
  extractObject(segmentation.response.clusters);
}
void ObjectManager::objectServerCallback(const std_msgs::Empty::ConstPtr& msg)
{
  //ROS_INFO("in objectserver callback");
  processScene();
}


void ObjectManager::updateObjectCallback(const object_manager::Object::ConstPtr& msg)
{
//  ROS_INFO("in update object callback");
  object_manager::Object new_obj = *msg;
//  object_list.clear();

  if(new_obj.inhand == false)
  {
    bool flag = true;

    for(unsigned int i = 0; i < object_list.size(); i++)
    {
      if(object_list[i].inhand == true)
      {
        object_list.erase(object_list.begin() + i);
        break;
      }
    }

//    processScene();
    while(flag && ros::ok()) {
    for(unsigned int i = 0; i < object_list.size(); i++)
    {
      ROS_INFO("i = %d",object_list[i].id);
      if(new_obj.color.r == object_list[i].color.r && new_obj.color.b == object_list[i].color.b)
      {
        flag = false;        
        object_list[i].header.stamp = ros::Time::now();
        object_list[i].id = new_obj.id;
        break;
      }
    }
    ros::spinOnce();
    ros::Duration(0.05f).sleep();
    ROS_INFO("Here");
    }
  }
  else {
//    processScene();
    for(unsigned int i = 0; i < object_list.size(); i++)
    {
      if(object_list[i].id== new_obj.id)
      {
        object_list[i].header.frame_id = new_obj.header.frame_id;
        object_list[i].header.stamp = ros::Time::now();
        object_list[i].pose = new_obj.pose;
        object_list[i].inhand = new_obj.inhand;
      }
    }
  }
}

bool ObjectManager::tableRequestCallback(object_manager::RequestTable::Request& request,object_manager::RequestTable::Response& response)
{
  response.table = table;  
  return true;
}

void ObjectManager::spin()
{
  object_manager::ObjectList objlist;
//  processScene();
  while(ros::ok())
  { 
//    processScene();
    objlist.object_list.resize(object_list.size());
    for(unsigned int i =0; i <object_list.size();i++)
      objlist.object_list[i] = object_list[i];

//    objlist.table = table;

    object_publisher.publish(objlist);        

    // publish markers
    publishObjectMarkers();
    drawTable(table);

    ros::spinOnce();
    ros::Duration(0.1f).sleep();
  }
}

/* Debugging function */
void ObjectManager::publishSegmentedPoints(std::vector<sensor_msgs::PointCloud2> clusters)
{
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::PointCloud2 final;

  final.height = 1;
  for(unsigned int i =0; i < clusters.size(); i++)
  {
    pc2 = clusters[i];
    final.header = pc2.header;    
    final.fields = pc2.fields;
    final.width += pc2.width;
    final.is_bigendian = pc2.is_bigendian;
    final.point_step = pc2.point_step;
    final.row_step += pc2.row_step;
    final.data.insert(final.data.end(),pc2.data.begin(),pc2.data.end());
  }
  point_publisher.publish(final);
}
