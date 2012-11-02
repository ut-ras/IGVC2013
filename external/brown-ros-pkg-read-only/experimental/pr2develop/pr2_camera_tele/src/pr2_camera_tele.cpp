#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ar_recog/Tags.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;
int* ready;

struct reqCartesianStruct {
    float px;
    float py;
    float pz;
    float ox;
    float oy;
    float oz;
    float ow;
} reqCartesianStruct_t;

struct calStruct {
    int calibrating;
    unsigned int min;
    unsigned int max;
    float scale;
    int minCount;
    int maxCount;
};

struct arTagInfo {
    int left_armID;
    int right_armID;
    int headID;
};

struct calStruct ar_cal;
struct reqCartesianStruct *ikRequest_right;
struct reqCartesianStruct *ikRequest_left;
struct arTagInfo ar_info;

// handles AR tag incoming data

void tag_handler(const ar_recog::TagsConstPtr &tags) {
    /* if no tags just return*/
    if (tags->tag_count == 0) {
        return;
    }

    int index = 0;
    unsigned int largest = 0;
    /* Checks for largest tag -> Removed b/c this is unnecessary*/
    /*
    for(unsigned int i=0; i < tags->tag_count; i++){
      if(tags->tags[i].id == 0){
        if(tags->tags[i].diameter > largest){
          largest = tags->tags[i].diameter;
          index = i;
        }
      }
      }*/

    //Don't know what this does.  But largest is always going to be zero now.  If nothing breaks, then get rid of this
    if (ar_cal.calibrating == 0) {
        if (largest > 30) {
            ar_cal.minCount = ar_cal.minCount + 1;
            ar_cal.min = (ar_cal.min * (ar_cal.minCount - 1) + largest) / ar_cal.minCount;
            if (ar_cal.minCount > 10) {
                ar_cal.max = ar_cal.min + 100;
                ar_cal.calibrating = 1;
            }
        }
        return;
    }

    ///cout << largest << " ";
    // if( largest >= ar_cal.min){
    // if(largest >ar_cal.max){
    //  largest = ar_cal.max;
    // }
    for (unsigned int i = 0; i < tags->tag_count; i++) {
        if (tags->tags[i].id == (unsigned int) ar_info.left_armID) {
            index = i;
            printf("x %d -- y %d -- diameter %d -- x rot %f -- y rot %f -- z rot %f\n",
                    tags->tags[index].x, tags->tags[index].y, tags->tags[index].diameter, tags->tags[index].xRot, tags->tags[index].yRot, tags->tags[index].zRot);
            if (tags->tags[index].diameter < 60 || tags->tags[index].diameter > 180) return; //acceptable bounds
            ikRequest_left->px = float(tags->tags[index].diameter) / 180.;
            ikRequest_left->py = 2. * (float(tags->tags[index].x) / float(tags->image_width) - .5); // because it's the left arm
            ikRequest_left->pz = -1. * (float(tags->tags[index].y) / float(tags->image_height) - .5);
            ikRequest_left->ox = -.8 * (tags->tags[index].zRot);
            ikRequest_left->oy = -.8 * (tags->tags[index].xRot);
            ikRequest_left->oz = .8 * (tags->tags[index].yRot);
            ikRequest_left->ow = 1.;
            *ready = 1;
            printf("left camera results: %f -- %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_left->px, ikRequest_left->py, ikRequest_left->pz, ikRequest_left->ox, ikRequest_left->oy, ikRequest_left->oz, ikRequest_left->ow);

        } else if (tags->tags[i].id == (unsigned int) ar_info.right_armID) {
            index = i;
            printf("x %d -- y %d -- diameter %d -- x rot %f -- y rot %f -- z rot %f\n",
                    tags->tags[index].x, tags->tags[index].y, tags->tags[index].diameter, tags->tags[index].xRot, tags->tags[index].yRot, tags->tags[index].zRot);
            if (tags->tags[index].diameter < 60 || tags->tags[index].diameter > 180) return; //acceptable bounds
            ikRequest_right->px = float(tags->tags[index].diameter) / 180.;
            ikRequest_right->py = 2. * (float(tags->tags[index].x) / float(tags->image_width) - .5) - .2; // because it's the right arm
            ikRequest_right->pz = -1. * (float(tags->tags[index].y) / float(tags->image_height) - .5);
            ikRequest_right->ox = -.8 * (tags->tags[index].zRot);
            ikRequest_right->oy = -.8 * (tags->tags[index].xRot);
            ikRequest_right->oz = .8 * (tags->tags[index].yRot);
            ikRequest_right->ow = 1.;
            *ready = 1;
            printf("right camera results: %f -- %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_right->px, ikRequest_right->py, ikRequest_right->pz, ikRequest_right->ox, ikRequest_right->oy, ikRequest_right->oz, ikRequest_right->ow);
        }
    }

}

void fillRequest(kinematics_msgs::GetPositionIK::Request* req, reqCartesianStruct *pos) {
    //this method fills the ik request packet (req) with the desired pose (pos)
    req->ik_request.pose_stamped.pose.position.x = pos->px;
    req->ik_request.pose_stamped.pose.position.y = pos->py;
    req->ik_request.pose_stamped.pose.position.z = pos->pz;
    req->ik_request.pose_stamped.pose.orientation.x = pos->ox;
    req->ik_request.pose_stamped.pose.orientation.y = pos->oy;
    req->ik_request.pose_stamped.pose.orientation.z = pos->oz;
    req->ik_request.pose_stamped.pose.orientation.w = pos->ow;
}

int main(int argc, char **argv) {
    ready = (int*) malloc(sizeof (int));
    *ready = 0;

    ros::init(argc, argv, "pr2_camera_tele");
    ros::NodeHandle n;
    ros::Publisher rightArmPub = n.advertise<trajectory_msgs::JointTrajectory > ("r_arm_controller/command", 100);
    ros::Publisher leftArmPub = n.advertise<trajectory_msgs::JointTrajectory > ("l_arm_controller/command", 100);


    n.getParam("/pr2_camera_tele/r_arm_id", ar_info.right_armID);
    n.getParam("/pr2_camera_tele/l_arm_id", ar_info.left_armID);

    ros::Subscriber tag = n.subscribe<ar_recog::Tags > ("tags", 1, &tag_handler);

    ar_cal.calibrating = 1;
    ar_cal.min = 60;
    ar_cal.max = 200;
    ar_cal.minCount = 0;
    ar_cal.scale = .08 / (ar_cal.max - ar_cal.min);

    std::vector<std::string> right_names(7);
    right_names[0] = "r_shoulder_pan_joint";
    right_names[1] = "r_shoulder_lift_joint";
    right_names[2] = "r_upper_arm_roll_joint";
    right_names[3] = "r_elbow_flex_joint";
    right_names[4] = "r_forearm_roll_joint";
    right_names[5] = "r_wrist_flex_joint";
    right_names[6] = "r_wrist_roll_joint";

    std::vector<std::string> left_names(7);
    left_names[0] = "l_shoulder_pan_joint";
    left_names[1] = "l_shoulder_lift_joint";
    left_names[2] = "l_upper_arm_roll_joint";
    left_names[3] = "l_elbow_flex_joint";
    left_names[4] = "l_forearm_roll_joint";
    left_names[5] = "l_wrist_flex_joint";
    left_names[6] = "l_wrist_roll_joint";


/*    if (argc < 8) {
        ROS_ERROR("Need input for each position");
        printf("usage: posx posy posz orientx orienty orientz orientw\n");
        return -1;
    }
*/
    ikRequest_right = (reqCartesianStruct*) malloc(sizeof (reqCartesianStruct));
    ikRequest_right->px = .75;//atof(argv[1]); //.75
    ikRequest_right->py = -.188;//atof(argv[2]); //-.188
    ikRequest_right->pz = 0;// atof(argv[3]); // 0
    ikRequest_right->ox = 0;//atof(argv[4]); // 0
    ikRequest_right->oy = 0;//atof(argv[5]); // 0
    ikRequest_right->oz = 0;//atof(argv[6]); // 0
    ikRequest_right->ow = 1;//atof(argv[7]); // 1
    printf("initial pos: %f -- %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_right->px, ikRequest_right->py, ikRequest_right->pz, ikRequest_right->ox, ikRequest_right->oy, ikRequest_right->oz, ikRequest_right->ow);


    ikRequest_left = (reqCartesianStruct*) malloc(sizeof (reqCartesianStruct));
    ikRequest_left->px = .75;//atof(argv[1]); //.75
    ikRequest_left->py = .188;//atof(argv[2]); //-.188
    ikRequest_left->pz = 0;//atof(argv[3]); // 0
    ikRequest_left->ox = 0;//atof(argv[4]); // 0
    ikRequest_left->oy = 0;//atof(argv[5]); // 0
    ikRequest_left->oz = 0;//atof(argv[6]); // 0
    ikRequest_left->ow = 1;//atof(argv[7]); // 1
    printf("initial pos: %f -- %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_left->px, ikRequest_left->py, ikRequest_left->pz, ikRequest_left->ox, ikRequest_left->oy, ikRequest_left->oz, ikRequest_left->ow);
    ros::Rate loop_rate(30);
    while (ar_cal.calibrating == 0) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
    ros::ServiceClient right_query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo > ("pr2_right_arm_kinematics/get_ik_solver_info");
    ros::ServiceClient right_ik_client = n.serviceClient<kinematics_msgs::GetPositionIK > ("pr2_right_arm_kinematics/get_ik");

    ros::service::waitForService("pr2_left_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik");
    ros::ServiceClient left_query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo > ("pr2_left_arm_kinematics/get_ik_solver_info");
    ros::ServiceClient left_ik_client = n.serviceClient<kinematics_msgs::GetPositionIK > ("pr2_left_arm_kinematics/get_ik");

    cout << "Initialized services" <<endl;
    // define the service messages
    kinematics_msgs::GetKinematicSolverInfo::Request r_kin_request;
    kinematics_msgs::GetKinematicSolverInfo::Response r_kin_response;

    //get the kinematic info about the joint for initialize the kinematic solver
    if (right_query_client.call(r_kin_request, r_kin_response)) {
        for (unsigned int i = 0; i < r_kin_response.kinematic_solver_info.joint_names.size(); i++) {
            ROS_DEBUG("Joint: %d %s", i, r_kin_response.kinematic_solver_info.joint_names[i].c_str());
        }
    } else {
        ROS_ERROR("Could not call query service");
        ros::shutdown();
        exit(1);
    }

    kinematics_msgs::GetKinematicSolverInfo::Request l_kin_request;
    kinematics_msgs::GetKinematicSolverInfo::Response l_kin_response;

    //get the kinematic info about the joint for initialize the kinematic solver
    if (left_query_client.call(l_kin_request, l_kin_response)) {
        for (unsigned int i = 0; i < l_kin_response.kinematic_solver_info.joint_names.size(); i++) {
            ROS_DEBUG("Joint: %d %s", i, l_kin_response.kinematic_solver_info.joint_names[i].c_str());
        }
    } else {
        ROS_ERROR("Could not call query service");
        ros::shutdown();
        exit(1);
    }

    //initialize right arm request packet
    kinematics_msgs::GetPositionIK::Request r_gpik_req;
    r_gpik_req.timeout = ros::Duration(5.0);
    r_gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";
    r_gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";

    r_gpik_req.ik_request.ik_seed_state.joint_state.position.resize(r_kin_response.kinematic_solver_info.joint_names.size());
    r_gpik_req.ik_request.ik_seed_state.joint_state.name = r_kin_response.kinematic_solver_info.joint_names;
    for (unsigned int i = 0; i < r_kin_response.kinematic_solver_info.joint_names.size(); i++) {
        r_gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (r_kin_response.kinematic_solver_info.limits[i].min_position + r_kin_response.kinematic_solver_info.limits[i].max_position) / 2.0;
    }

    //initialize left arm request packet
    kinematics_msgs::GetPositionIK::Request l_gpik_req;
    l_gpik_req.timeout = ros::Duration(5.0);
    l_gpik_req.ik_request.ik_link_name = "l_wrist_roll_link";
    l_gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";

    l_gpik_req.ik_request.ik_seed_state.joint_state.position.resize(l_kin_response.kinematic_solver_info.joint_names.size());
    l_gpik_req.ik_request.ik_seed_state.joint_state.name = l_kin_response.kinematic_solver_info.joint_names;
    for (unsigned int i = 0; i < l_kin_response.kinematic_solver_info.joint_names.size(); i++) {
        l_gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (l_kin_response.kinematic_solver_info.limits[i].min_position + l_kin_response.kinematic_solver_info.limits[i].max_position) / 2.0;
    }

    double* r_position = (double*) malloc(sizeof (double) *7);
    double* l_position = (double*) malloc(sizeof (double) *7);

    while (ros::ok()) {
        trajectory_msgs::JointTrajectoryPoint *r_jtp = new trajectory_msgs::JointTrajectoryPoint();
        trajectory_msgs::JointTrajectoryPoint *l_jtp = new trajectory_msgs::JointTrajectoryPoint();

        //puts the newest tag info into the request packet
        fillRequest(&r_gpik_req, ikRequest_right);
        fillRequest(&l_gpik_req, ikRequest_left);

        // get the right response packet
        kinematics_msgs::GetPositionIK::Response r_gpik_res;

        if (right_ik_client.call(r_gpik_req, r_gpik_res)) {
            if (r_gpik_res.error_code.val == r_gpik_res.error_code.SUCCESS) {
                for (unsigned int i = 0; i < r_gpik_res.solution.joint_state.name.size(); i++) {
                    //ROS_INFO("Joint: %s %f", r_gpik_res.solution.joint_state.name[i].c_str(), r_gpik_res.solution.joint_state.position[i]);
                    r_position[i] = r_gpik_res.solution.joint_state.position[i];
                }
            } else {
                ROS_ERROR("Right Inverse kinematics failed");
                //continue;
            }
        } else {
            ROS_ERROR("Right Inverse kinematics service call failed");
            //continue;
        }

        // get the left response packet
        kinematics_msgs::GetPositionIK::Response l_gpik_res;

        if (left_ik_client.call(l_gpik_req, l_gpik_res)) {
            if (l_gpik_res.error_code.val == l_gpik_res.error_code.SUCCESS) {
                for (unsigned int i = 0; i < l_gpik_res.solution.joint_state.name.size(); i++) {
                    //ROS_INFO("Joint: %s %f", l_gpik_res.solution.joint_state.name[i].c_str(), l_gpik_res.solution.joint_state.position[i]);
                    l_position[i] = l_gpik_res.solution.joint_state.position[i];
                }
            } else {
                ROS_ERROR("Left Inverse kinematics failed");
                //continue;
            }
        } else {
            ROS_ERROR("Left Inverse kinematics service call failed");
            //continue;
        }
        //printf("%f -- %f -- %f -- %f -- %f -- %f -- %f\n", position[0], position[1], position[2], position[3], position[4], position[5], position[6]);

        trajectory_msgs::JointTrajectory *r_traj = new trajectory_msgs::JointTrajectory();

        r_traj->joint_names = right_names;
        //jtp.positions = new vector<double>();
        //jtp.velocities = new vector<double>();
        for (int i = 0; i < 7; i++) {
            r_jtp->positions.push_back(r_position[i]);
            r_jtp->velocities.push_back(0.0);
        }
        //jtp.accelerations.push_back(0);
        //jtp.time_from_start.push_back(ros::Duration(0));
        r_traj->points.push_back(*r_jtp);
        rightArmPub.publish(*r_traj);

        trajectory_msgs::JointTrajectory *l_traj = new trajectory_msgs::JointTrajectory();

        l_traj->joint_names = left_names;
        //jtp.positions = new vector<double>();
        //jtp.velocities = new vector<double>();
        for (int i = 0; i < 7; i++) {
            l_jtp->positions.push_back(l_position[i]);
            l_jtp->velocities.push_back(0.0);
        }
        //jtp.accelerations.push_back(0);
        //jtp.time_from_start.push_back(ros::Duration(0));
        l_traj->points.push_back(*l_jtp);
        leftArmPub.publish(*l_traj);
        *ready = 0;

        delete l_jtp;
        delete r_jtp;
        ros::spinOnce();
        loop_rate.sleep();
        //printf("waiting...\n");
        //while(*ready == 0){}
    }
    delete r_position;
    delete ready;
    ros::shutdown();
    return 1;
}



