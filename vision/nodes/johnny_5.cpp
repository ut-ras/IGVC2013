#include <ros/ros.h>
#include <image_transport/image_transport.h> //publish and subscribe image

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/opencv.hpp"

#define DEBUG_LANES    1
#define DEBUG_DETAILED 0
#define STD_DEV_FACTOR_BLUE  2.0
#define STD_DEV_FACTOR_RED   2.1
#define STD_DEV_FACTOR_GREEN 2.2

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_GRAY[] = "Grey Window";

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

                //gpu::GpuMat downsampled;
                gpu::GpuMat equalized;

                //gpu::resize(imageFromCam, downsampled, Size(160, 120));

                //initialize all gpu matrices
                gpu::GpuMat blue_thresh, green_thresh, red_thresh, gray;
                vector<gpu::GpuMat> bgr_split;

                //split into BGR channels to which will be used to obtain
                //grayscaled image
                gpu::split(imageFromCam, bgr_split);

                Scalar mean_blue, stdDev_blue, mean_red, stdDev_red, mean_green, stdDev_green;
                gpu::meanStdDev(bgr_split[0], mean_blue , stdDev_blue);
                gpu::meanStdDev(bgr_split[1], mean_red  , stdDev_red);
                gpu::meanStdDev(bgr_split[2], mean_green, stdDev_green);

                double blue_lim, green_lim, red_lim;
                blue_lim  = mean_blue.val[0]  + (STD_DEV_FACTOR_BLUE  * stdDev_blue.val[0]);
                red_lim   = mean_red.val[0]   + (STD_DEV_FACTOR_RED   * stdDev_red.val[0]);
                green_lim = mean_green.val[0] + (STD_DEV_FACTOR_GREEN * stdDev_green.val[0]);

                gpu::threshold(bgr_split[0], blue_thresh,  blue_lim,  255, THRESH_BINARY);
                gpu::threshold(bgr_split[1], red_thresh,   red_lim,   255, THRESH_BINARY);
                gpu::threshold(bgr_split[2], green_thresh, green_lim, 255, THRESH_BINARY);

                gpu::GpuMat temp, thresholded;
                gpu::bitwise_and(blue_thresh, red_thresh, temp);
                gpu::bitwise_and(temp, green_thresh, thresholded);

                //LUCAS'S CODE STARTS HERE
                gpu::GpuMat gproc_image, gthresh_0, gthresh_1, gthresh_2, gthresh_3, blur, gred_orange, hsv, dilated_orange;
                vector<gpu::GpuMat> hsv_split;

                gpu::cvtColor(imageFromCam, hsv, CV_BGR2HSV);
                gpu::cvtColor(imageFromCam, gray, CV_BGR2GRAY);
                //Apply lucas blur
                gpu::GaussianBlur(gray, blur, Size(13, 13), 5.0, 5.0);

                gpu::split(hsv, hsv_split);

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

                gpu::equalizeHist(gray, equalized);

                if (DEBUG_LANES)
                    imshow("Equalized", (Mat)equalized);

                gpu::dilate(gred_orange, dilated_orange, Mat::ones(5, 11, CV_8U));
                if (DEBUG_LANES)
                    imshow("Dilated", (Mat)dilated_orange);

                gpu::GpuMat final;
                gpu::subtract(thresholded, dilated_orange, final);

                if (DEBUG_DETAILED) {
                    imshow("Blue", (Mat)blue_thresh);
                    waitKey(30);
                }
                if (DEBUG_DETAILED) {
                    imshow("Green", (Mat)green_thresh);
                    waitKey(30);
                }
                if (DEBUG_DETAILED) {
                    imshow("Red", (Mat)red_thresh);
                    waitKey(30);
                }
                if (DEBUG_LANES) {
                    imshow("Thresholded", (Mat)thresholded);
                    waitKey(30);
                }
                if (DEBUG_LANES) {
                    imshow("Final", (Mat)final);
                    waitKey(30);
                }

                image_pub_.publish(cv_ptr->toImageMsg());
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("caught cv_bridge exception: %s", e.what());
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "johnny_5");
    LaneDetector ld;
    ros::spin();
    return 0;
}
