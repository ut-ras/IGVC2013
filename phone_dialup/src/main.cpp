#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <cstdio>

#include "dialup.h"

ros::Publisher gtopic;
ros::Publisher mtopic;
ros::Publisher atopic;

double ntohd(unsigned char *data) {
    // Big Endian -> Small Endian
    // Then assumes IEEE double
    union {
        unsigned char bits[8];
        double value;    
    } packed;
    
    for (int i=0; i<8; i++)
        packed.bits[7-i] = data[i];

    return packed.value;
}

void gpub(unsigned char *buffer) {
    geometry_msgs::Point data;
    data.x = ntohd(buffer+0);
    data.y = ntohd(buffer+8);
    data.z = ntohd(buffer+16);
    gtopic.publish(data);
}

void mpub(unsigned char *buffer) {
    geometry_msgs::Vector3 data;
    data.x = ntohd(buffer+0);
    data.y = ntohd(buffer+8);
    data.z = ntohd(buffer+16);
    mtopic.publish(data);
}

void apub(unsigned char *buffer) {
    geometry_msgs::Vector3 data;
    data.x = ntohd(buffer+0);
    data.y = ntohd(buffer+8);
    data.z = ntohd(buffer+16);
    atopic.publish(data);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "phone_dialup");
    ros::NodeHandle node;

    gtopic = node.advertise<geometry_msgs::Point>("phone/gps", 5);
    mtopic = node.advertise<geometry_msgs::Vector3>("phone/mag", 5);
    atopic = node.advertise<geometry_msgs::Vector3>("phone/accel", 5);

    Dialup dialup(1024);
    unsigned char buffer[32];

    while(ros::ok()) {
        int len = dialup.read(buffer, 32);
        if (len != 25) continue; // Discard bad packets

        switch(buffer[0]) {
        case 'g': // GPS data
            gpub(buffer+1);
            break;
        case 'm': // Magnometer data
            mpub(buffer+1);
            break;
        case 'a': // Accelerometer data
            apub(buffer+1);
            break;
        }
        // Discard any packets that don't match
    }
}
