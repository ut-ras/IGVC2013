#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>

using namespace std;

ros::NodeHandle *n1;
void tagsCallback(const ar_recog::TagsConstPtr& msg);

