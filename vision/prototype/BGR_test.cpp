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

class LaneDetector {

    public:
        LaneDetector () {
        }
        
        ~LaneDetector () {
        }

        void laneDetectorCallback () {
            try {
                ROS_INFO("In Callback");
                
                //get image from cam
                const gpu::GpuMat& imageFromCam = (gpu::GpuMat) imread("/home/granny/ros/ros-pkg/IGVC2013/vision/prototype/Johnny5_BGR_Test.png", CV_LOAD_IMAGE_COLOR);;

                //initialize all gpu matrices
                gpu::GpuMat two_blue, gray;
                vector<gpu::GpuMat> bgr_split;
                
                //split into BGR channels to which will be used to obtain
                //grayscaled image
                gpu::split(imageFromCam, bgr_split);

                //add blue to itself to get two*blue
                gpu::add(bgr_split[0], bgr_split[0], two_blue);
                gpu::subtract(two_blue, bgr_split[1], gray);

                if (DEBUG_LANES) {
                    imshow("Blue", (Mat)bgr_split[0]);
                    imshow("Green", (Mat)bgr_split[1]);
                    imshow("Red", (Mat)bgr_split[2]);
                    imshow("Gray", (Mat)gray);
                    waitKey(30);
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("caught cv_bridge exception: %s", e.what());
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "johnny_5");
    LaneDetector ld;
    while (true) {
        ld.laneDetectorCallback();
    }
    ros::spin();
    return 0;
}
