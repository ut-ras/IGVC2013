#include <ros/ros.h>
#include <image_transport/image_transport.h> //publish and subscribe image

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/opencv.hpp"

#define DEBUG_LANES 0

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_GRAY[] = "Grey Window";
static const char WINDOW_BLUR[] = "Blur Window";
static const char WINDOW_CANN[] = "Canny Window";
static const char WINDOW_WHTE[] = "White Window";
static const char WINDOW_LAST[] = "Last Window";
static const char WINDOW_HOUG[] = "Hough Window";

//constants for gaussian blurr
static const Size blurSize = Size(5, 5);
static const double sigma1 = 5.0;
static const double sigma2 = 5.0;

//constants for canny
static const int lowThreshold = 50;
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
                gpu::GpuMat gray, blur, canny, hough, final, sat, hsv, val, white, dilated;
                //a vector to store the hsv channels
                vector<gpu::GpuMat> hsv_split;

                //get grascale image
                gpu::cvtColor(imageFromCam, gray, CV_BGR2GRAY);
                if (DEBUG_LANES) {
                    imshow(WINDOW_GRAY, (Mat)gray);
                    waitKey(30);
                }

                //convert out BGR image to HSV
                gpu::cvtColor(imageFromCam, hsv, CV_BGR2HSV);
                //split into hsv channels
                gpu::split(hsv, hsv_split);

                //filter all pixels with saturation more than 10
                gpu::threshold(hsv_split[1], sat,   10, 255, THRESH_BINARY_INV);
                //filter all pixels with value more than 230
                gpu::threshold(hsv_split[2], val,  230, 255, THRESH_BINARY);

                //and the saturation and value filtered images
                gpu::bitwise_and(sat, val, white);

                /*
                   This at present seems like a hack. I am dialating the
                   anded image as some edge information is lost. This at present 
                   produces no extra lines in the one bagged file that we have.
                   However we could possibly deal with this by having less
                   restrictive cut-offs so that when we and it with canny we do
                   no end up loosing the edges of the white lines.
                 */
                gpu::dilate(white, dilated, Mat::ones(7, 7, CV_8U));
                
                //blur the gray scale image
                gpu::GaussianBlur(gray, blur, blurSize, sigma1, sigma2);
                if (DEBUG_LANES) {
                    imshow(WINDOW_BLUR, (Mat)blur);
                    waitKey(30);
                }
                //perform canny edge detection on the blurred image
                gpu::Canny(blur, canny, lowThreshold, lowThreshold*ratio, kernal_size);
                if (DEBUG_LANES) {
                    imshow(WINDOW_CANN, (Mat)canny);
                    waitKey(30);
                }
                //and the canny edge detection image and the 
                //dilated combination of the saturation and value
                //filtered image
                gpu::bitwise_and(canny, dilated, final);
                if (DEBUG_LANES) {
                    imshow(WINDOW_LAST, (Mat)final);
                    waitKey(30);
                }
                if (DEBUG_LANES) {
                    imshow(WINDOW_WHTE, (Mat)final);
                }    waitKey(30);

                //comment next few lines if using gpu houghlines function

                //vector that stores the lines from the HoughTransfrom function
                vector<Vec4i> lines;
                //Havent messed around with the parametes at all. 
                //Could be a good starting point if stuff breaks.
                HoughLinesP((Mat)final, lines, rho, theta, 50, 50, 10);
                
                //comment next few lines to using HoughLinesP function
                /*
                vector<Vec2f> lines;
                gpu::HoughLines(dilated, hough, rho, theta, 100, false, 0);
                gpu::HoughLinesDownload(hough, lines);
                */
                for (size_t i = 0; i < lines.size(); i++) {
                //comment next few lines if using HoughLinesP function
                //TODO: DOES NOT WORK AT PRESENT.
                /*
                    float rho_ = lines[i][0];
                    float theta_ = lines[i][1];

                    Point pt1, pt2;

                    double a = cos(theta_);
                    double b = sin(theta_);

                    double x0 = a*rho_;
                    double y0 = b*rho_;

                    pt1.x = cvRound(x0 + 1000 * (-b));
                    pt1.y = cvRound(y0 + 1000 * (a));
                    pt2.x = cvRound(x0 - 1000 * (-b));
                    pt2.y = cvRound(y0 - 1000 * (a));

                    clipLine(cv_ptr->image.size(), pt1, pt2);
                    line(cv_ptr->image, pt1, pt2, cvScalar(0, 0, 255), 3, CV_AA);
                */

                    //comment the next 2 lines out if using gpu houghlines function
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
