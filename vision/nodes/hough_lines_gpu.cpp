#include <ros/ros.h>
#include <image_transport/image_transport.h> //publish and subscribe image

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>
#include <stdio.h>
#include <cstring>
#include <fstream>

#define DEBUG_LANES     0
#define HOUGH_PARAM     "Hough Parameter Adjustment Window"
#define RHO             10
#define THRESHOLD       35
#define MAX_LINE_LEN    25
#define MAX_LINE_GAP    20
#define DEFAULT_PARAMS  {RHO, THRESHOLD, MAX_LINE_LEN, MAX_LINE_GAP}
#define PARAMS_PATH     "/home/ras/ros/ros-pkg/IGVC2013/vision/thresholds/hough"

using namespace std;
using namespace cv;
using namespace cv::gpu;

namespace enc = sensor_msgs::image_encodings;

//constants for HoughLines
static const float rho = 1.0;
static const float theta = CV_PI/180.0f;
static const int threshold_val = 100;

class LaneDetector {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int params[4];
    bool DISPLAY_OUTPUT;

    public:
        LaneDetector (ros::NodeHandle nh, int* param_list, bool testing) : nh_(nh), it_(nh_), DISPLAY_OUTPUT(testing) {
            image_pub_ = it_.advertise("/vision/hough_lines", 1);
            image_sub_ = it_.subscribe("/vision/thresholder_planks", 1, &LaneDetector::laneDetectorCallback, this);
           
            for(int i = 0; i < 4; i++) {
                params[i] = param_list[i];
            }

            if (DISPLAY_OUTPUT) {
                namedWindow(HOUGH_PARAM);
                createTrackbar("Rho"       , HOUGH_PARAM, &params[0], 60);
                createTrackbar("Threshold" , HOUGH_PARAM, &params[1], 100);
                createTrackbar("Max Line Length" , HOUGH_PARAM, &params[2], 200);
                createTrackbar("Max Line Gap" , HOUGH_PARAM, &params[3], 200);
            }
        }
        
        ~LaneDetector () {
        }

        void laneDetectorCallback (const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_8UC1);
                
                //get image from cam
                const gpu::GpuMat& imageFromCam = (gpu::GpuMat) cv_ptr->image;

                //initialize all gpu matrices

                gpu::GpuMat d_lines;//dst(imageFromCam.rows, imageFromCam.cols, CV_8UC3, Scalar(0, 0, 0));
                HoughLinesBuf d_buf;

                gpu::HoughLinesP(imageFromCam, d_lines, d_buf, params[0]/2.0f, theta, params[1], params[3]);

                vector<Vec4i> lines;
                if (!d_lines.empty()) {
                    lines.resize(d_lines.cols);
                    Mat h_lines(1, d_lines.cols, CV32C4, &lines[0]);
                    d_lines.download(h_lines);
                }

                for (size_t i = 0; i < lines.size(); ++i) {
                    Vec4i l = lines[i];
                    line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 3, CV_AA);
                }
                
                
                if (DISPLAY_OUTPUT) {
                    cout << "Thresholds: ";
                    for(int i = 0; i < 4; i++){
                        cout << params[i] << " ";
                    }
                    cout << "\r";
                    waitKey(30);
                }

                if (DEBUG_LANES) {
                    imshow("Lines", dst);
                    imshow("Original", (Mat)imageFromCam);
                    waitKey(30);
                }

                cv_bridge::CvImage out_msg;
                out_msg.header = msg->header;
                out_msg.encoding = enc::TYPE_8UC3;
                out_msg.image = dst;

                image_pub_.publish(out_msg.toImageMsg());
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("caught cv_bridge exception: %s", e.what());
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hough_lines");
    ros::NodeHandle nh("~");
    bool is_testing;
    nh.param<bool>("test", is_testing, false);
    int thresh[4] = DEFAULT_PARAMS;

    if (!is_testing) {
        string line;
        string file_path = PARAMS_PATH;
        ifstream file;
        file.open (file_path.c_str());
        if(file.is_open()){
            getline (file,line);
            cout << "Running Hough Lines with custom Params: "<< endl << line << endl;
            istringstream iss(line);
            for(int i = 0; i < 4; i++){
                string token;
                getline (iss, token, ' ');
                thresh[i] = atoi(token.c_str());
            }
        } else {
            cout << "Unable to open threshold file" << endl;
        }
    } else {
       cout << "Running param selector" << endl;
    }

    LaneDetector ld(nh, thresh, is_testing);
    ros::spin();
    return 0;
}
