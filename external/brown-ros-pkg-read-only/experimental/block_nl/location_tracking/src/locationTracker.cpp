#include <location_tracking/locationTracker.h>

locationTracker::locationTracker(int argc,char** argv)
{
    std::string service_name;

    // To process an add_block request.
    nh.param<std::string>("add_block_srv",service_name, "/blocknlp/lt_add_block");
    add_block_server = nh.advertiseService<locationTracker::RequestAddBlock::Request,locationTracker::RequestAddBlock::Response>(service_name, boost::bind(&locationTracker::addBlockCallback,this,_1,_2));

    // To process an move_block request.
    nh.param<std::string>("move_block_srv",service_name, "/blocknlp/lt_move_block");
    move_block_server = nh.advertiseService<locationTracker::RequestMoveBlock::Request,locationTracker::RequestMoveBlock::Response>(service_name, boost::bind(&locationTracker::moveBlockCallback,this,_1,_2));

    // To process an release_block request.
    nh.param<std::string>("release_block_srv",service_name, "/blocknlp/lt_release_block");
    release_block_server = nh.advertiseService<locationTracker::RequestReleaseBlock::Request,locationTracker::RequestReleaseBlock::Response>(service_name, boost::bind(&locationTracker::releaseBlockCallback,this,_1,_2));

    // To process an delete_block request.
    nh.param<std::string>("delete_block_srv",service_name, "/blocknlp/lt_delete_block");
    delete_block_server = nh.advertiseService<locationTracker::RequestDeleteBlock::Request,locationTracker::RequestDeleteBlock::Response>(service_name, boost::bind(&locationTracker::deleteBlockCallback,this,_1,_2));

}

void locationTracker::addBlockCallback(locationTracker::RequestAddBlock::Request& request,locationTracker::RequestAddBlock::Response& response)
{
    int block_id = blockOverlap(request.pose);
    if(block_id == -1)
    {
        block_location block;
        block.pose = request->pose;
        block.block_id = getUniqueBlockID();
        block.isMoving = false;
        std::string str = "/locatiorTracker_block_";
        //Need to check to see if this goes horribly wrong due to not updating block_id. was originally block_id, not block.block
        str.append(block.block_id);
        block.frame_id = str;
        block_locations.push_back(block);

    }
    else
    {
        return block_id;
    }
}

void locationTracker::moveBlockCallback(locationTracker::RequestMoveBlock::Request& request,locationTracker::RequestMoveBlock::Response& response)
{
    block_id = request->block_id;
    block_id_pose = checkBlockOverlap(request->pose);
    if(block_id_pose == -1)
    {
        //Block has moved update it
        updateBlock(block_id, true, block_id_pose);


    }
    else{
        updateBlock(block_id, true, block_id_pose);  //maybe false?
    }

}

void locationTracker::releaseBlockCallback(locationTracker::RequestReleaseBlock::Request& request,locationTracker::RequestReleaseBlock::Response& response)
{
    block_id = request->block_id;
    block_id_pose = checkBlockOverlap(request->pose);
    if(block_id_pose == -1)
    {
        //Block has moved update it
        updateBlock(block_id, false, block_id_pose);


    }
    else{
        updateBlock(block_id, false, block_id_pose);  //maybe false?
    }
}

void locationTracker::deleteBlockCallback(locationTracker::RequestDeleteBlock::Request& request,locationTracker::RequestDeleteBlock::Response& response)
{
    myBlock_id = request->block_id;
    for (int j = 0; j<block_locations.size(); j++)
    {
        if (block_locations.at(j).block_id == myBlock_id)
        {
            block_locations.at(j).erase;
            //block_locations.at(j).isMoving = moveUpdate;
            break;
        }
    }

}

int locationTracker::checkBlockOverlap(geometry_msgs::Pose pose)
{
    for (int i = 0; i<block_locations.size(); i++)
    {
        float dx = pose.position.x - block_locations.at(i).pose.position.x;
        float dy = pose.position.y - block_locations.at(i).pose.position.y;
        float dz = pose.position.z - block_locations.at(i).pose.position.z;
        if (dx*dx + dy*dy + dz*dz < 0.01*0.01)
        {
            return block_locations.at(i).block_id;
        }
    }
    return -1;
}

int locationTracker::getUniqueBlockID()
{
    std::vector<int> taken_ids;
    int i = 0;
    while (i < 1000)
    {
        bool isUnique = true;
        for (int j = 0; j<block_locations.size(); j++)
        {
            if (block_locations.at(j).block_id == i)
            {
                i++;
                isUnique = false;
                break;
            }
        }
        if (isUnique)
        {
            return i;
        }
    }
    return -1;
}

void locationTracker::updateBlock(int myBlock_id, bool moveUpdate, geometry_msgs::Pose newPose)
{
    //std::vector<int> taken_ids;
    for (int j = 0; j<block_locations.size(); j++)
    {
        if (block_locations.at(j).block_id == myBlock_id)
        {
            block_locations.at(j).pose = newPose;
            block_locations.at(j).isMoving = moveUpdate;
            break;
        }
}
