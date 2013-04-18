#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

class LidarToPointCloud {
    public:
        LidarToPointCloud();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

LidarToPointCloud::LidarToPointCloud() {
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &LidarToPointCloud::scanCallback, this);

    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("/cloud", 100, false);
}

void LidarToPointCloud::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan, cloud);

    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processing");

    LidarToPointCloud filter;

    ros::spin();

    return 0;
}
/*
void lidarinfo (const sensor_msgs::LaserScanPtr& msg) {

    double current_angle = msg->angle_min;
    sensor_msgs::PointCloud p();
    int size = msg->ranges.size();
    vector<geometry_msgs::Point32> ptsvec();
    for (int i = 0; i < size; i++)
    {
        geometry_msgs::Point32 newPoint();
        newPoint.x = msg->ranges[msg->ranges[i]] * cos(current_angle);
        newPoint.y = msg->ranges[msg->ranges[i]] * sin(current_angle);
        current_angle += msg->angle_increment;
        newPoint.z = 0;
        ptsvec.push_back(newPoint);
    }
    p.points = &ptsvec;
    point_pub.publish(p);
    */
