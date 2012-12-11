#include "ros/ros.h"
#include "ocean_server_imu/RawData.h"
#include "filters/RotatedIMUData.h"
#include "math.h"

void callback(const ocean_server_imu::RawData::ConstPtr& msg) {
    float x_acc = msg->acceleration.x;
    float y_acc = msg->acceleration.y;
    float z_acc = msg->acceleration.z;
    filters::RotatedIMUData pub_msg;

    pub_msg.roll = msg->roll;
    pub_msg.pitch = msg->pitch;
    pub_msg.yaw = msg->yaw;

    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_filter");

    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("imu_data", 1000, callback);
    
    ros::spin();

    return 0;
}

