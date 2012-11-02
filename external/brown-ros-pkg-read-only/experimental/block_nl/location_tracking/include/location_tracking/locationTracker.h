
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#ifndef LOCATIONTRACKER_H
#define LOCATIONTRACKER_H

struct block_location
{
    int block_id;
    std::vector<int> RGB;
    tf::Pose pose;
    tf::frame_id parent_id;
    tf::frame_id frame_id;
    bool isMoving;
};


class locationTracker
{
public:
    locationTracker();
    virtual ~locationTracker();

    addBlockCallback(locationTracker::RequestAddBlock::Request& request,locationTracker::RequestAddBlock::Response& response);
    moveBlockCallback(locationTracker::RequestMoveBlock::Request& request,locationTracker::RequestMoveBlock::Response& response);
    releaseBlockCallback(locationTracker::RequestReleaseBlock::Request& request,locationTracker::RequestReleaseBlock::Response& response);
    deleteBlockCallback(locationTracker::RequestDeleteBlock::Request& request,locationTracker::RequestDeleteBlock::Response& response);

private:
    ros::NodeHandle nh;
    ros::ServiceServer add_block_server;
    ros::ServiceServer move_block_server;
    ros::ServiceServer release_block_server;
    ros::ServiceServer delete_block_server;

    block_location *block_locations;
};

#endif // LOCATIONTRACKER_H
