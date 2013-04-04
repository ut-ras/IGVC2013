#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <cv.h>
//#include <highgui.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <geometry_msgs/Twist.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class Decider
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
ros::Publisher heading_pub_;

double speed;

public:
    Decider() : it_(nh_)
    {
        image_pub_ = it_.advertise("/vision/decision", 1);
        heading_pub_ = nh_.advertise<geometry_msgs::Twist>("heading", 1);
        image_sub_ = it_.subscribe("/vision/out", 1, &Decider::imageCb, this);

        speed = 1.0;
        namedWindow("Input from vision vision");
        namedWindow("Output from vision decision");
    }

    ~Decider()
    {
        destroyWindow("Input from vision vision");
        destroyWindow("Output from vision decision");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
    
        //Convert input into usable arguments
        Mat src;        
        try
        {
            src = cv_bridge::toCvCopy(msg, enc::MONO8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        src = src * 255;

        gpu::GpuMat gsrc = (gpu::GpuMat) src;
        //Since we grabbed a binary image, it's on a 0-255 scale. This means we can't percieve the difference.
        //So threshold it and now we have what we want.
        //gpu::threshold(gsrc, gsrc, 0, 255, THRESH_BINARY);

        /* Equivalate this code
        ang_z = 0
        x = 0
        for i in range(input_image.rows):
            y = -(input_image.cols / 2)
            row = cv.GetRow(thresh_0,i)
            for j in row.tostring():
                ang_z = ang_z + (x * y *ord(j))
                y = y + 1
            x = x + 1
        ang_z = (ang_z * pi * 2 * 2 * 4 / 255 / input_image.rows / input_image.rows / input_image.cols / input_image.cols)
        p = Twist()
        p.linear.x = self.speed
        p.angular.z = ang_z
        self.pub.publish(p)*/

        imshow("Input from vision vision", src);
        waitKey(30);

        double ang_z = 0;
        int x = 0, y = 0; //White pixels on each side
        gpu::GpuMat gsum, gsum2;
    
        //gpu::columnSum(gsrc, gsum);
        //gpu::integral(gsrc,gsum);
        //gpu::Laplacian(gsrc, gsum, 1);
        /*vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        vector<Point> centers;

        gpu::findContours( gsrc, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

        Moments m = gpu::moments(contours, true);
        for (int i = 0; i < m.size().width; i++)
            for (int k = 0; k < m.size().height; k++)
                centers.push_back(Point(m.m01/m.m00, m.m10/m.m00));*/
        gpu::erode(gsrc, gsum, Mat::ones(8, 8, CV_8U));
        gpu::dilate(gsum, gsum2, Mat::ones(8, 8, CV_8U));

        imshow("Output from vision decision", (Mat) gsum2);
        waitKey(30);
        
        ang_z = (ang_z * 3.14 * 2 * 2 * 4 / 255); //Blah blah do some math accordingly

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = speed; 
        base_cmd.angular.z = ang_z;

        heading_pub_.publish(base_cmd);
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vision_decision");

    Decider ic;
    ros::spin();

    return 0;
}
