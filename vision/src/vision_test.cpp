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

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class ImageConverter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

public:
    ImageConverter() : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/out", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

        //Going to follow Frank's pycode as close as possible
        namedWindow("Input Video");
        //namedWindow("Red-Orange Video");
        //namedWindow("White Video");
        namedWindow("Red-Orange and White Video");
        //namedWindow("Equalized BGR");
    }

    ~ImageConverter()
    {
        destroyWindow("RAW");
        destroyWindow("OUTPUT");
        destroyWindow("Input Video");
        destroyWindow("Red-Orange Video");
        destroyWindow("White Video");
        destroyWindow("Red-Orange and White Video");
        destroyWindow("Equalized BGR");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //Main processing node

        gpu::GpuMat gOut;
    
        //Convert input into usable arguments
        Mat src;        
        try
        {
            src = cv_bridge::toCvCopy(msg, enc::BGR8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        gpu::GpuMat gSrc = (gpu::GpuMat) src;
        gpu::GpuMat gblur_image;
        gpu::GpuMat gproc_image;


        imshow("Input Video",(Mat)gSrc);
        waitKey(30);

        //PROCESSING TIME!! YAAAY~~~
        //Following Frank's steps so far (this is his ported code)
        gpu::GaussianBlur(gSrc, gblur_image, Size(7,7), 1.5, 1.5);
        gpu::cvtColor(gblur_image, gproc_image, CV_BGR2HSV);

        //Split into HSV channels and RGB channels too
        //Using Frank names and prepending g to all gpu mats

        vector<gpu::GpuMat> gsplit_bgr;
        gpu::split(gblur_image, gsplit_bgr);        

        vector<gpu::GpuMat> gsplit_image;
        gpu::split(gproc_image, gsplit_image);
        gpu::GpuMat gthresh_0, gthresh_1, gthresh_2;
        gpu::GpuMat gred_orange;

        gpu::threshold(gsplit_image[1], gthresh_0, 128, 255, THRESH_BINARY);// >50% saturation
        gpu::threshold(gsplit_image[0], gthresh_1, 220, 255, THRESH_BINARY);// > purple
        gpu::threshold(gsplit_image[0], gthresh_2,  10, 255, THRESH_BINARY_INV);//<Yellow-Orange

        gpu::add(gthresh_1, gthresh_2, gred_orange);
        gpu::bitwise_and(gred_orange, gthresh_0, gred_orange);


        gOut = gred_orange;

        //imshow("Red-Orange Video", (Mat)gred_orange);
        //waitKey(30);

        ///Begin white detection

        gpu::cvtColor(gblur_image, gproc_image, CV_BGR2HLS);
        gpu::split(gproc_image, gsplit_image);

        //Greater than 80% luminence
        gpu::threshold(gsplit_image[1], gthresh_0, 204, 255, THRESH_BINARY);

        //imshow("White Video", (Mat)gthresh_0);
        //waitKey(30);

        ///End white detection

        gpu::bitwise_or(gred_orange, gthresh_0, gthresh_0);
        imshow("Red-Orange and White Video", (Mat) gthresh_0);
        waitKey(30);

        //Experimental Histogram Equalization in three color channels
        //Update: Do not equalize HSV unless you like bad acid trips; it equalized the hue band. 
        gpu::GpuMat gEqB, gEqG, gEqR, gTotalBGR;
        gpu::equalizeHist(gsplit_bgr[0], gEqB);
        gpu::equalizeHist(gsplit_bgr[1], gEqG);
        gpu::equalizeHist(gsplit_bgr[2], gEqR);
        gpu::GpuMat gEqualizedBGR[3] = {gEqB, gEqG, gEqR};

        gpu::merge(gEqualizedBGR, 3, gTotalBGR);

        //imshow("Equalized BGR", (Mat) gTotalBGR);
        //waitKey(30);
        
        

        //Publish image results
        Mat outMat = (Mat)gthresh_0;
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        out_msg.image    = outMat;

        image_pub_.publish(out_msg.toImageMsg());
    }


   //TODO: debug strange behavior with other functions in class. Learn to pass GpuMats correctly.
    void DetectLanes(gpu::GpuMat &srcGray, Mat &dstBGR, int resolution, int minVotes)
    {
        vector<Vec2f> lines_;
        gpu::GpuMat vecLines;
        // gpu::HoughLines(srcGray, vecLines, resolution, CV_PI/180, minVotes);

        // gpu::HoughLinesDownload(vecLines, lines_);
        
        for ( size_t i = 0; i < lines_.size(); i++)
        {
            float rho = lines_[i][0];
            float theta = lines_[i][resolution];

            double a = cos(theta);
            double b = sin(theta);
            double x0 = a*rho;
            double y0 = b*rho;

            Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));

            clipLine(srcGray.size(), pt1, pt2);

            if (!dstBGR.empty())
                line( dstBGR, pt1, pt2, cvScalar(0,0,255), resolution, 8);
        }
    }

};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_converter");

    ImageConverter ic;
    ros::spin();

    return 0;
}

/*
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

static const char WINDOW[] = "Image window";

class ImageConverter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

public:
    ImageConverter() : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/out", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

        cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //cv_bridge::CvImagePtr cv_ptr, gray_ptr;
        Mat src, dst;        

        try
        {
            src = cv_bridge::toCvCopy(msg, enc::BGR8)->image;
            //gray_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        {
           cvtColor(cv_ptr->image, gray_ptr->image, CV_BGR2GRAY);
        }    
        
        cv::imshow(WINDOW, gray_ptr->image);
        cv::waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());

        
        namedWindow("Lanes:",1);
        namedWindow("Thresholded Lanes:", 1);

        //First grab feed
        cv::gpu::cvtColor(src, src, CV_BGR2GRAY);
        dst = src.clone();

        //Und zen vi process ja
        LaneMarkingsDetector(src, dst, 5);
        //DetectLanes(dst, dst, 10, 1);

        //Naechste werden wir den Computer kaput gemachen
        imshow("Lanes:", dst);
        threshold(dst,dst,180,255,THRESH_BINARY);

        imshow("Thresholded Lanes:", dst);

        cv::waitKey(30);

        //Now we need to set back the headers to publish this shit.
        //Fucking shitty out of date data types. Fuck me.

        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        out_msg.image    = dst; // Your cv::Mat

        image_pub_.publish(out_msg.toImageMsg());

        DetectEdges((gpu::GpuMat)src);
    }

    void LaneMarkingsDetector(cv::Mat &srcGray, cv::Mat &dstGray, int tau)
    {
        dstGray.setTo(0);

        int aux = 0;
        for (int j=0; j < srcGray.rows; ++j)
        {
            unsigned char *ptRowSrc = srcGray.ptr<uchar>(j);
            unsigned char *ptRowDst = dstGray.ptr<uchar>(j);

            for (int i=tau; i < srcGray.cols-tau; ++i)
            {
                if ( ptRowSrc[i] != 0)
                {
                    aux = 2*ptRowSrc[i];
                    aux += -ptRowSrc[i-tau];
                    aux += -ptRowSrc[i+tau];
                    aux += -abs((int)(ptRowSrc[i-tau] - ptRowSrc[i+tau]));

                    aux = (aux<0)?(0):(aux);
                    aux = (aux>255)?(255):(aux);

                    ptRowDst[i] = (unsigned char)aux;
                }
            }
        }
    }

    
    void DetectEdges(gpu::GpuMat in)
    {
        gpu::GpuMat edges;
        namedWindow("Basic Canny Edge Detection",1);
        for ever
        {
            gpu::cvtColor(in, edges, CV_BGR2GRAY);
            gpu::GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
            gpu::Canny(edges, edges, 0, 30, 3);
            imshow("Basic Canny Edge Detection", edges);
            if(waitKey(30) >= 0) break; //The ESC key
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    ImageConverter ic;
    // cv::namedWindow(WINDOW);
   
    //cv::waitKey(); 
    ros::spin();
    return 0;
}


//Stole this function from
//http://marcosnietoblog.wordpress.com/2011/12/27/lane-markings-detection-and-vanishing-point-detection-with-opencv/
//Input: Source image, destination image, expected lane width in pixels
//Note: Source and dest are grayscale only.



//I theoreticize that this function needs a binary image. Will test later if
//this comment is still here

//A Hough transform requires a thresholded binary image, assuming that is what is being done here
void DetectLanes(cv::Mat &srcGray, cv::Mat &dstBGR, int resolution, int minLength)
{
    vector<Vec2f> lines_;
    HoughLines(srcGray, lines_, resolution, CV_PI/180, minLength);

    for ( size_t i = 0; i < lines_.size(); i++)
    {
        float rho = lines_[i][0];
        float theta = lines_[i][resolution];

        double a = cos(theta);
        double b = sin(theta);
        double x0 = a*rho;
        double y0 = b*rho;

        Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));

        cv::clipLine(srcGray.size(), pt1, pt2);

        if (!dstBGR.empty())
            line( dstBGR, pt1, pt2, cvScalar(0,0,255), resolution, 8);

        cv::imwrite("HOUGH.bmp", dstBGR);
    }
}


//This function takes in an HSV image and thresholds it
IplImage* GetThresholdedImage(IplImage* imgHSV, bool isGray = false)
{   
    if (isGray)
    {
        IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
        cvInRangeS(imgHSV, cvScalar(200,200,200), cvScalar(255,255,255), imgThresh);
        return imgThresh;
    }
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(170,160,60), cvScalar(180,256,256), imgThresh); 
    return imgThresh;
} 

int main (int argc, char* argv[])
{

    ros::init(argc, argv, "image_converter");
    try
    {
        
        cv::Mat src_host = cv::imread("TESTIMAGE.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        cv::gpu::GpuMat dst, src;
        src.upload(src_host);

        cv::gpu::threshold(src, dst, 200.0, 255.0, CV_THRESH_BINARY);
        

        cv::Mat result_host = (cv::Mat) dst;
        cv::imshow("Result", result_host);
        cv::waitKey();
        
        VideoCapture cap(0);
        if(!cap.isOpened())
            return -1;

        Mat src, dst;
        namedWindow("Lanes:",1);
        namedWindow("Thresholded Lanes:", 1);
        for ever
        {
            //First grab feed
            cap >> src;
            cvtColor(src, src, CV_BGR2GRAY);
            dst = src.clone();

            //Und zen vi process ja
            LaneMarkingsDetector(src, dst, 5);
            //DetectLanes(dst, dst, 10, 1);

            //Naechste werden wir den Computer kaput gemachen
            imshow("Lanes:", dst);
            threshold(dst,dst,180,255,THRESH_BINARY);
    

            //TODO: Figure out why this function doesn't work.
            //I think I misunderstand the argument meanings.
            //Lucas - 5:39 Wednesday
            //DetectLanes(dst, dst, 10, 30);

            imshow("Thresholded Lanes:", dst);
            if(waitKey(30) >= 0) break; //The ESC key
        }
        //cv::imshow("Result", dst);        
        //cv::waitKey();
        ImageConverter ic;
        ros::spin();
    }
    catch(const cv::Exception& ex)
    {
        std::cout << "Error: " << ex.what() << std::endl;
    }
    return 0;
}
*/
