#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <utility>

#include <sensor_msgs/LaserScan.h>

#include "filters/Orientation.h"
#include "filters/PlanarData.h"

#include "vn_200_imu/vn_200_ins_soln.h"
#include "vn_200_imu/vn_200_accel_gyro_compass.h"

#define _USE_MATH_DEFINES
#define DISTANCE 0.5

ros::Publisher pub;

using namespace std;

double bound0to2Pi(const double angle) {
    return fmod((fmod(angle, (2.0 * M_PI)) + 2.0 * M_PI), (fmod(2.0, M_PI)));
}

bool myfunction(pair<double, double> i, pair<double, double> j) {
    return i.second < j.second;
}

void image_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Heard something in call back");
    
    filters::PlanarData msg;

    msg.startAngle = scan->angle_min;
    msg.angleRange = scan->angle_max - scan->angle_min;

    vector< pair<double, double> > rts;

    for(size_t i = 0; i < scan->ranges.size(); ++i) {
        double dist = scan->ranges[i];

        double angle = scan->angle_min + scan->angle_increment * i;

        double px = dist * cos(angle) + DISTANCE;
        double py = dist * sin(angle);

        double radius = sqrt(py * py + px * px);
        double theta  = atan2(py, px);
        
        pair<double, double> p(radius, theta);
        rts.push_back(p);
    }
    
    sort(rts.begin(), rts.end(), myfunction);

    for(size_t i = 0; i < scan->ranges.size(); ++i) {
        msg.ranges.push_back(rts[i].first);
        msg.angles.push_back(rts[i].second);
    }

    //cout << "About to publish" << endl << msg

    pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_scan_transform");

    ros::NodeHandle n;

    pub = n.advertise<filters::PlanarData>("image_scan_transformed", 1000);
    ros::Subscriber sub = n.subscribe("image_scan", 1000, image_scan_callback);

    ROS_INFO("Subscriber to image_scan!");

    ros::spin();

    return 0;
}
