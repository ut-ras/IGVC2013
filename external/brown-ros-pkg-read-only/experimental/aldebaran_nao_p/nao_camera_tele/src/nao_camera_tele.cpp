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
#include <NAOcontrol/ArmPose.h>

using namespace std;
int* ready;

struct reqCartesianStruct {
    float px;
    float py;
    float pz;
    float ox;
    float oy;
    float oz;
    float hand;
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
    int right_handID;
    int headID;
};

struct calStruct ar_cal;
struct reqCartesianStruct *ikRequest_right;
struct reqCartesianStruct *ikRequest_left;
struct arTagInfo ar_info;
int r_hand_off;

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
            ikRequest_left->px = .02+ float(tags->tags[index].diameter) / 900.;
            ikRequest_left->py = 2. * (float(tags->tags[index].x) / float(tags->image_width) - .5); // because it's the left arm
            ikRequest_left->pz = -1. * (float(tags->tags[index].y) / float(tags->image_height) - .5);
            ikRequest_left->ox = -.8 * (tags->tags[index].zRot);
            ikRequest_left->oy = -.8 * (tags->tags[index].xRot);
            ikRequest_left->oz = .8 * (tags->tags[index].yRot);
            *ready = 1;
            printf("left camera results: %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_left->px, ikRequest_left->py, ikRequest_left->pz, ikRequest_left->ox, ikRequest_left->oy, ikRequest_left->oz);

        } else if (tags->tags[i].id == (unsigned int) ar_info.right_armID) {
            index = i;
            printf("x %d -- y %d -- distance %d -- x rot %f -- y rot %f -- z rot %f\n",
                    tags->tags[index].x, tags->tags[index].y, tags->tags[index].distance, tags->tags[index].xRot, tags->tags[index].yRot, tags->tags[index].zRot);
            if (tags->tags[index].distance < 200 || tags->tags[index].distance > 600) return; //acceptable bounds
            ikRequest_right->px = -float(tags->tags[index].distance - 200) / 3200. + .18;
            ikRequest_right->py = .2 * (float(tags->tags[index].x) / float(tags->image_width) - .5) - .1; // because it's the right arm
            ikRequest_right->pz = -.3 * (float(tags->tags[index].y) / float(tags->image_height) - .5);
            //ikRequest_right->ox = -.8 * (tags->tags[index].zRot);
            //ikRequest_right->oy = -.8 * (tags->tags[index].xRot);
            //ikRequest_right->oz = .8 * (tags->tags[index].yRot);
            *ready = 1;
            printf("right camera results: %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_right->px, ikRequest_right->py, ikRequest_right->pz, ikRequest_right->ox, ikRequest_right->oy, ikRequest_right->oz);
        } else if (tags->tags[i].id == (unsigned int) ar_info.right_handID) {
            if(r_hand_off > 3){
               ikRequest_right->hand = 0;
            }
            r_hand_off = 0;
        }
    }
}
void fillRequest(NAOcontrol::ArmPose* req, reqCartesianStruct *pos) {
    //this method fills the ik request packet (req) with the desired pose (pos)
    req->armPose[0] = pos->px;
    req->armPose[1] = pos->py;
    req->armPose[2] = pos->pz;
    req->armPose[3] = pos->ox;
    req->armPose[4] = pos->oy;
    req->armPose[5] = pos->oz;
    req->mask = 7;
    req->hand = pos->hand;
}

int main(int argc, char **argv) {
    ready = (int*) malloc(sizeof (int));
    *ready = 0;
    r_hand_off = 0;

    ros::init(argc, argv, "pr2_camera_tele");
    ros::NodeHandle n;
    ros::Publisher rightArmPub = n.advertise<NAOcontrol::ArmPose > ("RArm", 100);
    ros::Publisher leftArmPub = n.advertise<NAOcontrol::ArmPose > ("LArm", 100);


    n.getParam("/nao_camera_tele/r_arm_id", ar_info.right_armID);
    n.getParam("/nao_camera_tele/l_arm_id", ar_info.left_armID);
    n.getParam("/nao_camera_tele/r_hand_id", ar_info.right_handID);

    ros::Subscriber tag = n.subscribe<ar_recog::Tags > ("tags", 1, &tag_handler);

    ar_cal.calibrating = 1;
    ar_cal.min = 60;
    ar_cal.max = 200;
    ar_cal.minCount = 0;
    ar_cal.scale = .08 / (ar_cal.max - ar_cal.min);



/*    if (argc < 8) {
        ROS_ERROR("Need input for each position");
        printf("usage: posx posy posz orientx orienty orientz orientw\n");
        return -1;
    }
*/
    ikRequest_right = (reqCartesianStruct*) malloc(sizeof (reqCartesianStruct));
    ikRequest_right->px = .05;//atof(argv[1]); //.75
    ikRequest_right->py = -.1;//atof(argv[2]); //-.188
    ikRequest_right->pz = -0.04;// atof(argv[3]); // 0
    ikRequest_right->ox = 1.2;//atof(argv[4]); // 0
    ikRequest_right->oy = .57;//atof(argv[5]); // 0
    ikRequest_right->oz = .12;//atof(argv[6]); // 0
    printf("initial pos: %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_right->px, ikRequest_right->py, ikRequest_right->pz, ikRequest_right->ox, ikRequest_right->oy, ikRequest_right->oz);


    ikRequest_left = (reqCartesianStruct*) malloc(sizeof (reqCartesianStruct));
    ikRequest_left->px = .75;//atof(argv[1]); //.75
    ikRequest_left->py = .188;//atof(argv[2]); //-.188
    ikRequest_left->pz = 0;//atof(argv[3]); // 0
    ikRequest_left->ox = 0;//atof(argv[4]); // 0
    ikRequest_left->oy = 0;//atof(argv[5]); // 0
    ikRequest_left->oz = 0;//atof(argv[6]); // 0
    printf("initial pos: %f -- %f -- %f -- %f -- %f -- %f\n", ikRequest_left->px, ikRequest_left->py, ikRequest_left->pz, ikRequest_left->ox, ikRequest_left->oy, ikRequest_left->oz);
    ros::Rate loop_rate(6);
    while (ar_cal.calibrating == 0) {
        ros::spinOnce();
        loop_rate.sleep();
    }


    //double* r_position = (double*) malloc(sizeof (double) *7);
    //double* l_position = (double*) malloc(sizeof (double) *7);

    while (ros::ok()) {

        NAOcontrol::ArmPose r_arm;
        NAOcontrol::ArmPose l_arm;

        //puts the newest tag info into the request packet
        fillRequest(&r_arm, ikRequest_right);
        fillRequest(&l_arm, ikRequest_left);
        ikRequest_right->hand = 2;
        r_hand_off++;
        if(r_hand_off == 3){
            ikRequest_right->hand = 1;
        }

        // get the right response packet
        

        
        rightArmPub.publish(r_arm);

        
        leftArmPub.publish(l_arm);
        *ready = 0;

        //delete l_jtp;
        //delete r_jtp;

        printf("Got here\n");
        ros::spinOnce();
        loop_rate.sleep();
        printf("Finished loop\n");
        //printf("waiting...\n");
        //while(*ready == 0){}
    }
    //delete r_position;
    delete ready;
    ros::shutdown();
    return 1;
}



