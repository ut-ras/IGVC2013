#include <ros/ros.h>
#include <image_transport/image_transport.h> //publish and subscribe image

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/opencv.hpp"

#define DEBUG_LANES 1

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_GRAY[] = "Grey Window";
static const char WINDOW_BLUR[] = "Blur Window";
static const char WINDOW_CANN[] = "Canny Window";
static const char WINDOW_WHTE[] = "White Window";
static const char WINDOW_LAST[] = "Last Window";
static const char WINDOW_HOUG[] = "Hough Window";

//constants for gaussian blurr
static const Size blurSize = Size(9, 9);
static const Size blurSize2 = Size(11, 11);
static const double sigma1 = 5.0;
static const double sigma2 = 5.0;

//constants for canny
static const int lowThreshold = 20;
static const int ratio = 3;
static const int kernal_size = 3;

//constants for HoughLines
static const float rho = 1.0;
static const float theta = CV_PI/180;
static const int threshold_val = 100;

class LaneDetector {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
        LaneDetector () : it_(nh_) {
           image_pub_ = it_.advertise("image_lane", 1);
           image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &LaneDetector::laneDetectorCallback, this);
        }
        
        ~LaneDetector () {
        }

        void laneDetectorCallback (const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                ROS_INFO("In Callback");
                cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
                
                //get image from cam
                const gpu::GpuMat& imageFromCam = (gpu::GpuMat) cv_ptr->image;

                //initialize all gpu matrices
                gpu::GpuMat gray, blur, canny, hough, final, sat, hsv, val, white, dilated, hls, lum, lum_val_and;
                //a vector to store the hsv channels
                vector<gpu::GpuMat> hsv_split, hls_split;

                //get grascale image
                gpu::cvtColor(imageFromCam, gray, CV_BGR2GRAY);
                //convert out BGR image to HSV
                gpu::cvtColor(imageFromCam, hsv, CV_BGR2HSV);
                //convert out BGR image to HLS
                gpu::cvtColor(imageFromCam, hls, CV_BGR2HLS);
                //split into hsv channels
                gpu::split(hsv, hsv_split);
                gpu::split(hls, hls_split);


                gpu::threshold(hls_split[1], lum, 220, 255, THRESH_BINARY);

                if (DEBUG_LANES)
                    imshow("lum", (Mat)lum);

                //filter all pixels with value more than 230
                gpu::threshold(hsv_split[2], val,  230, 255, THRESH_BINARY);

                if (DEBUG_LANES)
                    imshow("val", (Mat)val);

                gpu::bitwise_and(val, lum, lum_val_and);


                //and the saturation and value filtered images

                /*
                   This at present seems like a hack. I am dialating the
                   anded image as some edge information is lost. This at present 
                   produces no extra lines in the one bagged file that we have.
                   However we could possibly deal with this by having less
                   restrictive cut-offs so that when we and it with canny we do
                   no end up loosing the edges of the white lines.
                 */
                gpu::dilate(lum_val_and, dilated, Mat::ones(5, 5, CV_8U));
                if (DEBUG_LANES)
                    imshow("dilated Lum and val", (Mat)dilated);
                
                //blur the gray scale image
                gpu::GaussianBlur(gray, blur, blurSize, sigma1, sigma2);
                
                //LUCAS'S CODE STARTS HERE

                gpu::GpuMat gproc_image, gthresh_0, gthresh_1, gthresh_2, gthresh_3, blur2, gred_orange;

                //Apply lucas blur
                gpu::GaussianBlur(gray, blur2, blurSize2, sigma1, sigma2);

                //TODO: Hardcoded threshold values to be replaced with dynamic thresholds
                gpu::threshold(hsv_split[2], gthresh_3, 210, 255, THRESH_BINARY);
                gpu::threshold(hsv_split[1], gthresh_0, 80, 255, THRESH_BINARY);
                gpu::threshold(hsv_split[0], gthresh_1, 210, 255, THRESH_BINARY);
                gpu::threshold(hsv_split[0], gthresh_2,  40, 255, THRESH_BINARY_INV);

                gpu::add(gthresh_1, gthresh_2, gred_orange);
                gpu::add(gred_orange, gthresh_3, gred_orange);
                gpu::bitwise_and(gred_orange, gthresh_0, gred_orange);

                if (DEBUG_LANES)
                    imshow("orange red", (Mat)gred_orange);
                //LUCAS'S CODE ENDS

                gpu::GpuMat orange_dilated, val_minus_orange;
                
                gpu::dilate(gred_orange, orange_dilated, Mat::ones(5, 9, CV_8U));
                if (DEBUG_LANES)
                    imshow("orange red dilated", (Mat)orange_dilated);
                gpu::subtract(lum_val_and, orange_dilated, val_minus_orange);

                if (DEBUG_LANES)
                    imshow("diff", (Mat)val_minus_orange);

                //perform canny edge detection on the blurred image
                //gpu::Canny(blur, canny, lowThreshold, lowThreshold*ratio, kernal_size, true);
                //gpu::Canny(lum_val_and, canny, lowThreshold, lowThreshold*ratio, kernal_size, true);
                if (DEBUG_LANES) {
                   // imshow(WINDOW_CANN, (Mat)canny);
                    waitKey(30);
                }
                //and the canny edge detection image and the 
                //dilated combination of the saturation and value
                //filtered image
                //gpu::bitwise_and(canny, val_minus_orange, final);
                //comment next few lines if using gpu houghlines function

                //vector that stores the lines from the HoughTransfrom function
                vector<Vec4i> lines;
                //Havent messed around with the parametes at all. 
                //Could be a good starting point if stuff breaks.
                HoughLinesP((Mat)val_minus_orange, lines, rho, theta, 50, 50, 10);
                
                //comment next few lines to using HoughLinesP function
                
                    //comment the next 2 lines out if using gpu houghlines function
                for (size_t i = 0; i < lines.size(); i++) {
                    Vec4i l = lines[i];
                    line(cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(225, 0, 0), 3, CV_AA);
                }
                if (DEBUG_LANES) {
                    imshow(WINDOW_HOUG, (Mat)cv_ptr->image);
                    waitKey(30);
                }

                image_pub_.publish(cv_ptr->toImageMsg());
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("caught cv_bridge exception: %s", e.what());
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_detector");
    LaneDetector ld;
    ros::spin();
    return 0;
}
