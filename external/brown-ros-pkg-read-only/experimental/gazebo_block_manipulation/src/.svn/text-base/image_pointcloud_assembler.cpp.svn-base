#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

pcl::PointCloud<pcl::PointXYZ> pointcloud;
bool pointcloud_is_fresh;
ros::Publisher cloudout;
std::string SUBSCRIBE_KINECT_FRAME = "/head_mount_kinect_rgb_optical_frame";
std::string PUBLISH_KINECT_FRAME = "/kinect_rgb_optical_frame";
std::string SUBSCRIBE_TOPIC_NAME = "/kinect_rgb/points";
std::string PUBLISH_TOPIC_NAME = "/kinect_head/camera/rgb/points";
std::string IMAGE_TOPIC_NAME = "/kinect_rgb/image_raw";

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, pointcloud);
    pointcloud_is_fresh = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::Image image = *msg;
    if (pointcloud_is_fresh)
    {
        long num_pixels = image.height * image.width;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;
        cloud_colored.header.frame_id = pointcloud.header.frame_id;
        cloud_colored.height = pointcloud.height;
        cloud_colored.width = pointcloud.width;
        cloud_colored.points.resize(pointcloud.points.size());
        for (int i = 0; i<num_pixels; i++)
        {
            long index_image = 3*i;
            cloud_colored.points[i].x = pointcloud.points[i].x;
            cloud_colored.points[i].y = pointcloud.points[i].y;
            cloud_colored.points[i].z = pointcloud.points[i].z;
            cloud_colored.points[i].r = image.data[index_image];
            cloud_colored.points[i].g = image.data[index_image+1];
            cloud_colored.points[i].b = image.data[index_image+2];
        }
        cloudout.publish(cloud_colored);
    }
    pointcloud_is_fresh = false;
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        for (int i = 1; i<argc; i++)
        {
            std::string str = std::string(argv[i]);
            if (str.find("subscribe_pointcloud=") != std::string::npos)
            {
                str.erase(str.begin(), str.begin() + 21);
                SUBSCRIBE_TOPIC_NAME = str;
            }
            else if (str.find("publish_pointcloud=") != std::string::npos)
            {
                str.erase(str.begin(), str.begin() + 19);
                PUBLISH_TOPIC_NAME = str;
            }
            else if (str.find("subscribe_frame_id=")  != std::string::npos)
            {
                str.erase(str.begin(), str.begin() + 19);
                SUBSCRIBE_KINECT_FRAME = str;
            }
            else if (str.find("publish_frame_id=")  != std::string::npos)
            {
                str.erase(str.begin(), str.begin() + 17);
                PUBLISH_KINECT_FRAME = str;
            }
            else if (str.find("image=") != std::string::npos)
            {
                str.erase(str.begin(), str.begin() + 6);
                IMAGE_TOPIC_NAME = str;
            }

        }
    }
    //initialize the ROS node
    ros::init(argc, argv, "kinect_image_pointcloud_assembler");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pointcloud_is_fresh = false;
     ROS_INFO_STREAM("image_pointcloud_assembler: Subscribing to pointcloud XYZ topic: " << SUBSCRIBE_TOPIC_NAME);
    ros::Subscriber cloudin = nh.subscribe<sensor_msgs::PointCloud2>(SUBSCRIBE_TOPIC_NAME, 1, pointcloudCallback);

    ROS_INFO_STREAM("image_pointcloud_assembler: Subscribing to RGB8 image topic: " << IMAGE_TOPIC_NAME);

    image_transport::Subscriber imagein = it.subscribe(IMAGE_TOPIC_NAME, 1, imageCallback);

    ROS_INFO_STREAM("image_pointcloud_assembler: Publishing to pointcloud XYZRGB topic: " << PUBLISH_TOPIC_NAME);
    cloudout = nh.advertise<sensor_msgs::PointCloud2> (PUBLISH_TOPIC_NAME, 1);
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    ROS_INFO_STREAM("image_pointcloud_assembler: Subscribing to frame_id: " << SUBSCRIBE_KINECT_FRAME);
    ROS_INFO_STREAM("image_pointcloud_assembler: Publishing to frame_id: " << PUBLISH_KINECT_FRAME);
    while(ros::ok())
    {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), SUBSCRIBE_KINECT_FRAME, PUBLISH_KINECT_FRAME));
    ros::spinOnce();
    ros::Duration(0.05).sleep();
    }
}
