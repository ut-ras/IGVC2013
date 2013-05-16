#include <ros/ros.h>
#include <image_transport/image_transport.h> //publish and subscribe image

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>
#include <stdio.h>
#include <cstring>
#include <fstream>

#define DEBUG_LANES     1
#define HOUGH_PARAM     "Hough Parameter Adjustment Window"
#define RHO             10
#define THRESHOLD       35
#define MAX_LINE_NUM    1
#define DEFAULT_PARAMS  {RHO, THRESHOLD, MAX_LINE_NUM}
#define PARAMS_PATH     "/home/granny/ros/ros-pkg/IGVC2013/vision/thresholds/hough"

using namespace cv;
using namespace std;

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
    int params[3];
    bool DISPLAY_OUTPUT;

    public:
        LaneDetector (ros::NodeHandle nh, int* param_list, bool testing) : nh_(nh), it_(nh_), DISPLAY_OUTPUT(testing) {
            image_pub_ = it_.advertise("/vision/hough_lines", 1);
            image_sub_ = it_.subscribe("/vision/thresholder_lane", 1, &LaneDetector::laneDetectorCallback, this);
           
            for(int i = 0; i < 3; i++) {
                params[i] = param_list[i];
            }

            if (DISPLAY_OUTPUT) {
                namedWindow(HOUGH_PARAM);
                createTrackbar("Rho"       , HOUGH_PARAM, &params[0], 60);
                createTrackbar("threshold" , HOUGH_PARAM, &params[1], 100);
                createTrackbar("Num Lines" , HOUGH_PARAM, &params[2], 10);
            }
        }
        
        ~LaneDetector () {
        }

        void getHoughLines (const gpu::GpuMat& image, vector<Vec2f>& lines_gpu) {
            gpu::GpuMat d_lines;
            gpu::HoughLines(image, d_lines, params[0]/2.0f, theta, params[1], true, params[2]);

            if (!d_lines.empty()) {
                //lines_gpu.resize(d_lines.cols);
                //Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
                gpu::HoughLinesDownload(d_lines, lines_gpu);
            }
        }

        void drawLines (Mat& image, vector<Vec2f>& lines_gpu) {
            for (size_t i = 0; i < lines_gpu.size(); ++i) {
                float r = lines_gpu[i][0];
                float t = lines_gpu[i][1];
                Point pt1, pt2;
                double a = cos(t), b = sin(t);
                double x0 = a*r, y0 = b*r;
                pt1.x = cvRound(x0 + 1000 * (-b));
                pt1.y = cvRound(y0 + 1000 * (a));
                pt2.x = cvRound(x0 - 1000 * (-b));
                pt2.y = cvRound(y0 - 1000 * (a));

                line(image, pt1, pt2, Scalar(255), 3, CV_AA);
            }
        }

        void laneDetectorCallback (const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_8UC1);
                
                //get image from cam
                const gpu::GpuMat& imageFromCam = (gpu::GpuMat) cv_ptr->image;

                //initialize all gpu matrices

                Mat dst(imageFromCam.rows, imageFromCam.cols, CV_8UC3, Scalar(0, 0, 0));

                /*
                vector<Vec4i> lines;
                HoughLinesP((Mat)imageFromCam, lines, params[0]/2.0f, theta, params[1], params[2], params[3]);
                for (size_t i = 0; i < lines.size(); ++i) {
                    Vec4i l = lines[i];
                    line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 3, CV_AA);
                }
                */
                
                vector<Vec2f> lines_gpu_left;
                vector<Vec2f> lines_gpu_right;

                Range all_rows  (0, imageFromCam.rows - 1);
                Range left_cols (0, (imageFromCam.cols/2));
                Range right_cols((imageFromCam.cols/2) + 1, imageFromCam.cols - 1);

                gpu::GpuMat left (imageFromCam, all_rows, left_cols);
                gpu::GpuMat right(imageFromCam, all_rows, right_cols);

                Mat final_left (imageFromCam.rows, imageFromCam.cols/2, CV_8UC1);
                Mat final_right(imageFromCam.rows, imageFromCam.cols/2, CV_8UC1);

                getHoughLines(left, lines_gpu_left);
                drawLines(final_left, lines_gpu_left);

                getHoughLines(right, lines_gpu_right);
                drawLines(final_right, lines_gpu_right);

                if (DISPLAY_OUTPUT) {
                    cout << "Thresholds: ";
                    for(int i = 0; i < 3; i++){
                        cout << params[i] << " ";
                    }
                    cout << "\r";
                    waitKey(30);
                }

                if (DEBUG_LANES) {
                    imshow("Lines", dst);
                    imshow("Original", (Mat)imageFromCam);
                    imshow("Left",  (Mat)final_left);
                    imshow("Right", (Mat)final_right);
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
    int thresh[3] = DEFAULT_PARAMS;

    if (!is_testing) {
        string line;
        string file_path = PARAMS_PATH;
        ifstream file;
        file.open (file_path.c_str());
        if(file.is_open()){
            getline (file,line);
            cout << "Running Hough Lines with custom Params: "<< endl << line << endl;
            istringstream iss(line);
            for(int i = 0; i < 3; i++){
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
