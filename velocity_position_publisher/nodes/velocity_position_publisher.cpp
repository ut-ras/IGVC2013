#include "ros/ros.h"
#include "imu_filter/FilteredIMUData.h"
#include "velocity_position_publisher/velocity_position_info.h"

double x_vel, y_vel, z_vel;
double x_pos, y_pos, z_pos;

int last_time_nsec, last_time_sec;

ros::Publisher pub;

void callback(const imu_filter::FilteredIMUData::ConstPtr& msg) {
    velocity_position_publisher::velocity_position_info pub_msg;

    pub_msg.roll = msg->roll;
    pub_msg.pitch = msg->pitch;
    pub_msg.yaw = msg->yaw;

    pub_msg.acceleration.x = msg->acceleration.x;
    pub_msg.acceleration.y = msg->acceleration.y;
    pub_msg.acceleration.z = msg->acceleration.z;
    
    int time_sec = msg->header.stamp.sec;
    int time_nsec = msg->header.stamp.nsec;

    if (last_time_sec == -1 || last_time_nsec == -1) {
        last_time_nsec = time_nsec;
        last_time_sec = time_sec;
    }

    int delta_t_sec = time_sec - last_time_sec;
    int delta_t_nsec = time_nsec - last_time_nsec;

    uint actual_change;

    if (delta_t_sec != 0){
        actual_change =((uint) (1000000000 - last_time_nsec) + time_nsec);
    } else {
        actual_change = delta_t_nsec;
    }

    double change = actual_change * 0.000000001;
    
    pub_msg.velocity.x = x_vel + (pub_msg.acceleration.x * change);
    pub_msg.velocity.y = y_vel + (pub_msg.acceleration.y * change);
    pub_msg.velocity.z = z_vel + (pub_msg.acceleration.z * change);

    pub_msg.position.x = x_pos + (x_vel * change) + (0.5 * pub_msg.acceleration.x * change * change);
    pub_msg.position.y = y_pos + (y_vel * change) + (0.5 * pub_msg.acceleration.y * change * change);
    pub_msg.position.z = z_pos + (z_vel * change) + (0.5 * pub_msg.acceleration.z * change * change);


    ROS_INFO("x_vel: %f\ny_vel: %f\nz_vel: %f\nx_pos: %f\ny_pos: %f\nz_pos: %f\n", pub_msg.velocity.x, pub_msg.velocity.y, pub_msg.velocity.z, pub_msg.position.x, pub_msg.position.y, pub_msg.position.z);

    last_time_nsec = time_nsec;
    last_time_sec = time_sec;
    
    x_pos = pub_msg.position.x;
    y_pos = pub_msg.position.y;
    z_pos = pub_msg.position.z;

    x_vel = pub_msg.velocity.x;
    y_vel = pub_msg.velocity.y;
    z_vel = pub_msg.velocity.z;

}
int main (int argc, char **argv) {
    
    x_vel = x_pos = y_vel = y_pos = z_vel = z_pos = 0.0;

    last_time_sec = last_time_nsec = -1;

    ros::init(argc, argv, "velocity_position_info");
    
    ros::NodeHandle n;
    
    pub = n.advertise<velocity_position_publisher::velocity_position_info>("velocity_position_publisher", 1000);
    ros::Subscriber sub = n.subscribe("/imu_filtered_data", 1000, callback);

    ros::spin();

    return 0;
}
