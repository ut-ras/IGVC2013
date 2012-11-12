#include "ros/ros.h"
#include "ocean_server_imu/RawData.h"
#include "imu_filter/FilteredIMUData.h"
#include "math.h"

float x_vel;
float y_vel;
float z_vel;

int last_time_nsec;
int last_time_sec;

ros::Publisher pub;

void callback(const ocean_server_imu::RawData::ConstPtr& msg){
    imu_filter::FilteredIMUData pub_msg;

    pub_msg.roll = msg->roll;
    pub_msg.pitch = msg->pitch;
    pub_msg.yaw = msg->yaw;

    float cos_phi = cos(-pub_msg.roll);
    float cos_theta = cos(-pub_msg.pitch);
    float cos_psi = cos(pub_msg.yaw); 
    
    float sin_phi = sin(-pub_msg.roll);
    float sin_theta = sin(-pub_msg.pitch);
    float sin_psi = sin(pub_msg.yaw);

    float x_acc = msg->acceleration.x;
    float y_acc = msg->acceleration.y;
    float z_acc = msg->acceleration.z;

    float final_x = x_acc * (cos_theta * cos_psi) + y_acc * (sin_phi * sin_theta * cos_psi - cos_phi * sin_psi) - z_acc * (sin_phi * sin_psi + cos_phi * sin_theta * cos_psi) ;
    float final_y = x_acc*(cos_theta*sin_psi) + y_acc*(cos_phi*cos_psi + sin_phi*sin_theta*sin_psi) - z_acc*(cos_phi*sin_theta*sin_psi - sin_phi * cos_psi);
    float final_z = x_acc*(-sin_theta) + y_acc*(sin_phi*cos_theta) - z_acc*(cos_phi*cos_theta);
    /*
    int time_sec = msg->header.stamp.sec;
    int time_nsec = msg->header.stamp.nsec;

    if (last_time_sec == -1 || last_time_nsec == -1) {
        last_time_nsec = time_nsec;
        last_time_sec = time_sec;
    }

    int delta_t_sec = last_time_sec - time_sec;
    int delta_t_nsec = last_time_nsec - time_nsec;

    uint actual_change;

    if (delta_t_sec > 0){
        actual_change =((uint) (1000000000 - last_time_nsec) + time_nsec);
    } else {
        actual_change = time_nsec - last_time_nsec;
    }
    
    ROS_INFO("old time: %d.%d; new time: %d.%d; change: %u", last_time_sec, last_time_nsec, time_sec, time_nsec, actual_change);

    last_time_nsec = time_nsec;
    last_time_sec = time_sec;
    */
    
    pub_msg.acceleration.x = final_x;
    pub_msg.acceleration.y = final_y;
    pub_msg.acceleration.z = final_z + 9.81;
    pub_msg.header = msg->header;

    pub.publish(pub_msg);
    
}

int main(int argc, char **argv) {
    x_vel = 0.0;
    y_vel = 0.0;
    z_vel = 0.0;

    last_time_sec = -1;
    last_time_nsec = -1;

    ros::init(argc, argv, "imu_filter");

    ros::NodeHandle n;
    
    pub = n.advertise<imu_filter::FilteredIMUData>("imu_filtered_data", 1000);
    ros::Subscriber sub = n.subscribe("imu_data", 1000, callback);
    
    ros::spin();

    return 0;
}

