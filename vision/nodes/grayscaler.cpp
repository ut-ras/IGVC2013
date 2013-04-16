#include <ros/ros.h>
#include <image_transport/image_transport.h> //publish and subscribe image

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class Grayscaler {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
        Grayscaler () : it_(nh_) {
            image_pub_ = it_.advertise("image_gray", 1);
            image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &Grayscaler::imageCb, this);
            
            cv::namedWindow(WINDOW);
        }

        ~Grayscaler() {
            cv::destroyWindow(WINDOW);
        }

        void imageCb (const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            image_pub_.publish(cv_ptr->toImageMsg());
        }
};

int main(int argc, char **argv) {

    //initialize node
    ros::init(argc, argv, "grayscaler");
    Grayscaler g;
    ros::spin();
    return 0;
}
