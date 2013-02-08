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
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

void test_cuda() {
    try {
        cv::Mat src_host = cv::imread("TESTIMAGE.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        cv::gpu::GpuMat dst, src;
        src.upload(src_host);

        cv::gpu::threshold(src, dst, 128.0, 255.0, CV_THRESH_TOZERO);

        cv::Mat result_host = (cv::Mat) dst;
        cv::imshow("Result", result_host);
        cv::waitKey();
    } catch(const cv::Exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    }
}

void test_opencv() {
    try {
        cv::Mat dst,
                src = cv::imread("TESTIMAGE.jpg", CV_LOAD_IMAGE_GRAYSCALE);

        cv::threshold(src, dst, 128.0, 255.0, CV_THRESH_TOZERO);

        cv::imshow("Result", dst);
        cv::waitKey();
    } catch(const cv::Exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    }
}


//callback function
void mouseEvent(int evt, int x, int y, int flags, void* param)
{
    if(evt==CV_EVENT_LBUTTONDOWN)
    {
        printf("%d %d\n",x,y);
    }
}

void test_mouse_interaction() {
    try {
        cvNamedWindow("MyWindow");

        //assigning the callback function for mouse events
        cvSetMouseCallback("MyWindow", mouseEvent, 0);

        //load and display an image
        IplImage* img = cvLoadImage("TESTIMAGE.jpg");
        cvShowImage("MyWindow", img);
        
        //wait for key press
        cvWaitKey(0);
      
        //cleaning up
        cvDestroyWindow("MyWindow");
        cvReleaseImage(&img);

    } catch(const cv::Exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    }
}


void box_draw_mouse_callback( int event, int x, int y, int flags, void* param );

CvRect box;
bool drawing_box = false;

void draw_box( IplImage* img, CvRect rect ){
        cvRectangle( img, cvPoint(box.x, box.y), cvPoint(box.x+box.width,box.y+box.height),
                                cvScalar(0xff,0x00,0x00) );
}

// Implement mouse callback
void box_draw_mouse_callback( int event, int x, int y, int flags, void* param ){
        IplImage* image = (IplImage*) param;

            switch( event )
            {
                case CV_EVENT_MOUSEMOVE: 
                if( drawing_box ){
                    box.width = x-box.x;
                    box.height = y-box.y;
                }
                break;

                case CV_EVENT_LBUTTONDOWN:
                drawing_box = true;
                box = cvRect( x, y, 0, 0 );
                break;

                case CV_EVENT_LBUTTONUP:
                drawing_box = false;
                if( box.width < 0 ){
                    box.x += box.width;
                    box.width *= -1;
                }
                if( box.height < 0 ){
                    box.y += box.height;
                    box.height *= -1;
                }
                draw_box( image, box );

                //get the BGR values of the pixels in selection,B=[0], G=[1], R[2]
                cv::Mat img = cv::imread("TESTIMAGE.jpg");

                std::cout << "Top left coordinate: " << box.x << ", " << box.y << std::endl;
                std::cout << "Bottom right coordinate: " << box.x+box.width << ", " << box.y+box.height << std::endl;

                for(int i=box.x; i<box.x+box.width; i++)
                    for(int j=box.y; j<box.y+box.height; j++)
                      std::cout << "Value at " << i << ", " << j << " " << img.at<cv::Vec3b>(i,j)[0] << " " << img.at<cv::Vec3b>(i,j)[1] << " " << img.at<cv::Vec3b>(i,j)[2] << std::endl;
                break;
                                                                                                                                                        }
}


void test_box_draw()
{
    const char* name = "Box Example";
    box = cvRect(-1,-1,0,0);

    IplImage* image = cvLoadImage( "TESTIMAGE.jpg" );
//    IplImage* temp = cvCloneImage( image );

    cvNamedWindow( name );

    // Set up the callback
    cvSetMouseCallback( name, box_draw_mouse_callback, (void*) image);

    // Main loop
    while( 1 )
    {
        //cvCopy( image, temp );

        if( drawing_box ) 
            draw_box( image, box );

        cvShowImage( name, image );

        if( cvWaitKey( 15 )==27 ) 
            break;
    }

    cvReleaseImage( &image );
//    cvReleaseImage( &temp );
    cvDestroyWindow( name );
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "opencv_cuda_test");
    //test_cuda();
    // test_opencv();
    //test_mouse_interaction();
    test_box_draw();
}


