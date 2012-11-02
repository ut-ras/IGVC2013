#include <ros/ros.h>
#include <std_msgs/Int8.h>"
#include <std_msgs/Float32.h>"
#include <std_msgs/Bool.h>"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>


#include <tf/transform_listener.h>

int width = -1;
int height = -1;
image_transport::Publisher pic_pub;
ros::Publisher period_pub;
ros::Publisher cloud_pub;
ros::Publisher sync_pub;
ros::Publisher spose_pub;
ros::Publisher pose_pub;
ros::Publisher marker_pub;

int mode = 0;
double bestDistance = -1;
double period = 33.183990478515625;
ros::Time bestTime;
IplImage* compImage;
IplImage* targetImage;
ros::Time targetTime;
sensor_msgs::PointCloud cloud;

tf::TransformListener *tfstream;

geometry_msgs::PoseStamped lastPose;
double distances[5];
int disIdx = 0;

void pointCallback(const sensor_msgs::PointCloudConstPtr& msg) {
  if (mode != 3) return;

  cloud.points = msg->points;
  cloud.header = msg->header;
  cloud.channels = msg->channels;

  mode = 4;
}

void thresh() {
  double threshold = (double) 16.0;
  std::cout << threshold << std::endl;
  cvSmooth(compImage,compImage,CV_MEDIAN);
  cvThreshold(compImage,compImage,threshold,255,CV_THRESH_BINARY);
  cvDilate(compImage,compImage,NULL,8);
  
  sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(compImage, "mono8");
  pic_pub.publish(out);

  sensor_msgs::PointCloud tmp;

  tmp.header = cloud.header;
  for (int i = 0; i < cloud.points.size(); i++) {
    int x = cloud.channels[2].values[i];
    int y = cloud.channels[1].values[i];
    int val = ((uchar*)(compImage->imageData + y*compImage->widthStep))[x];
    if (val > 0) {
      tmp.points.push_back(cloud.points[i]);
    }
  }

  sensor_msgs::PointCloud tmp2;

  try {
    tfstream->transformPointCloud("/base_link",cloud.header.stamp,tmp,"/base_link",tmp2);
  } catch(tf::TransformException ex) {
    std::cout << ex.what() << std::endl;
    return;
  }

  sensor_msgs::PointCloud obj;

  obj.header = tmp2.header;
  int highest = 0;
  for (int i = 0; i < tmp2.points.size(); i++) {
    float z = tmp2.points[i].z;
    if (z > tmp2.points[highest].z) {
      highest = i;
    }
  }
  double total = 0;
  double ax = 0;
  double ay = 0;
  double az = 0;
  for (int i = 0; i < tmp2.points.size(); i++) {
    double x = tmp2.points[i].x;
    double y = tmp2.points[i].y;
    double z = tmp2.points[i].z;
    if (fabs(tmp2.points[highest].z - z) < .05 && fabs(tmp2.points[highest].x - x) < .1 && fabs(tmp2.points[highest].y - y) < .1) {
      ax += x;
      ay += y;
      az += z;
      obj.points.push_back(tmp2.points[i]);
      total++;
    }
  }

  ax /= total;
  ay /= total;
  az /= total;

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = obj.header;
  pose.pose.pose.position.x = ax;
  pose.pose.pose.position.y = ay;
  pose.pose.pose.position.z = az;

  geometry_msgs::PoseStamped spose;
  spose.header = obj.header;
  spose.pose.position.x = ax;
  spose.pose.position.y = ay;
  spose.pose.position.z = az;

  visualization_msgs::Marker marker;
  marker.header = obj.header;
  marker.ns = "target";
  marker.id = 24005;
  marker.type = 2;
  marker.pose.position.x = ax;
  marker.pose.position.y = ay;
  marker.pose.position.z = az;
  marker.scale.x = .05;
  marker.scale.y = .05;
  marker.scale.z = .05;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.5,0);

  double distance = 0;
  distance += (spose.pose.position.x-lastPose.pose.position.x)*(spose.pose.position.x-lastPose.pose.position.x);
  distance += (spose.pose.position.y-lastPose.pose.position.y)*(spose.pose.position.y-lastPose.pose.position.y);
  distance += (spose.pose.position.z-lastPose.pose.position.z)*(spose.pose.position.z-lastPose.pose.position.z);

  int good = 0;
  for (int i = 0; i < 5; i++) {
    if (distance <= 1.1*distances[i]) good++;
  }
  lastPose = spose;
  distances[disIdx] = distance;
  disIdx = (disIdx+1) % 5;

  bool quad = false;
  if (spose.pose.position.x < .8 && spose.pose.position.y > -0.2) quad = true;

  if (good >= 3) {
    marker.color.r = 0.0;
    if (quad) {
      marker.color.g = 1.0;
      cloud_pub.publish(obj);
      pose_pub.publish(pose);
      spose_pub.publish(spose);
    } else {
      marker.color.b = 1.0;
    }
  }

  marker_pub.publish(marker);
}

void controlCallback(const std_msgs::Int8::ConstPtr& msg) {
  mode = (int) msg->data;
  std::cout << mode << std::endl;
}

void handleFrame(const sensor_msgs::ImageConstPtr& msg) {
  if (mode == 0) return;

  if (mode == 1 || mode == 3) {
    if (width != -1) {
	    cvReleaseImage(&targetImage);
	    cvReleaseImage(&compImage);
    }

    width = msg->width;
    height = msg->height;

	  compImage = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
    cvSetZero(compImage);

	  targetImage = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        int color = msg->data[x+y*msg->step];
        ((uchar *)(targetImage->imageData + y*targetImage->widthStep))[x]= color;
      }
    }

    targetTime = msg->header.stamp;
    bestDistance = -1;
    if (mode != 3) {
      mode = 0;
    }
  }

  if (mode == 2) {
    double distance = 0;
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        int a = msg->data[x+y*msg->step];
        int b = ((uchar*)(targetImage->imageData + y*targetImage->widthStep))[x];
        distance += (a-b)*(a-b);
      }
    }
    if (distance < bestDistance || bestDistance == -1) {
      bestDistance = distance;
      for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
          int color = msg->data[x+y*msg->step];
          ((uchar *)(compImage->imageData + y*compImage->widthStep))[x]= color;
        }
      }

      bestTime = msg->header.stamp;
      std_msgs::Float32 period_msg;
      period = (bestTime - targetTime).toSec();
      period_msg.data = period;
      period_pub.publish(period_msg);

      sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(compImage, "mono8");
      pic_pub.publish(out);

    }

  }

  if (mode == 4) {
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        int a = msg->data[x+y*msg->step];
        int b = ((uchar*)(targetImage->imageData + y*targetImage->widthStep))[x];
        ((uchar *)(compImage->imageData + y*compImage->widthStep))[x]= abs(a-b);
      }
    }

    sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(compImage, "mono8");
    pic_pub.publish(out);

    std_msgs::Bool now;
    now.data = true;
    sync_pub.publish(now);
    thresh();
    
    std::cout << "sync" << std::endl;

    mode = 3;
  }

	//sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(copy, "mono8");
	//pub.publish(out);

	//cvReleaseImage(&copy);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "period");
  ros::NodeHandle nh;
  ros::Subscriber cntrl = nh.subscribe("/period/detect",1000,controlCallback);
  //ros::Subscriber thresh = nh.subscribe("/period/threshold",10000,thresholdCallback);
  ros::Subscriber points = nh.subscribe("/narrow_stereo/points",1000,pointCallback);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/narrow_stereo/left/image_mono",1,handleFrame);
  pic_pub = it.advertise("/period/image_mono",1);
  period_pub = nh.advertise<std_msgs::Float32>("period/period",1000);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("period/cloud",1000);
  sync_pub = nh.advertise<std_msgs::Bool>("period/sync",1000);
  spose_pub = nh.advertise<geometry_msgs::PoseStamped>("period/simple_pose",1000);
  pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("period/pose",1000);
  marker_pub = nh.advertise<visualization_msgs::Marker>("period/marker",1000);
  tf::TransformListener listener;
  tfstream = &listener;
  ros::spin();
}
