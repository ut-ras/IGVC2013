#include <ros/ros.h>
#include <iostream>

#include <naoExpmnt/QVGA.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;




int main(int argc, char** argv) {
	cvNamedWindow( "naocam", 1 );

	ros::init(argc, argv, "watcher");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<naoExpmnt::QVGA>("qvga");

	IplImage* img = cvCreateImage(cvSize(640,480), 8, 3);
	//IplImage* img = cvCreateImage(cvSize(160,120), 8, 3);
	
	while(1) {
		naoExpmnt::QVGA srv;
		srv.request.ignore = 0;
		if (client.call(srv)) {
			for( int y=0; y < img->height; y++ ) {
				uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
				for( int x=0; x <img->width; x++ ) {
					ptr[3*x+2] = srv.response.rgb[y/4*img->width/4*3+3*x/4+0];//R
					ptr[3*x+1] = srv.response.rgb[y/4*img->width/4*3+3*x/4+1];//G
					ptr[3*x+0] = srv.response.rgb[y/4*img->width/4*3+3*x/4+2];//B
				}
			}
			cvShowImage( "naocam", img );
		}
		if (cvWaitKey(66) == 27) break;
	}

	cvReleaseImage(&img);
}
