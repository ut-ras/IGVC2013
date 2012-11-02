#include <ros/ros.h>
#include <NaoImageTransport/Naoimage.h>
#include <iostream>
#include <vector>
#include <stdio.h>

#include <opencv/highgui.h>
//#include <../../build/opencv-svn/3rdparty/include/jpeglib.h>
#include <jpeglib.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#define WIDTH 160
#define HEIGHT 120
#define CHANNELS 3
#define SIZE 76800

int size;
JOCTET src[SIZE];
JSAMPLE image[WIDTH*HEIGHT*CHANNELS];
JSAMPROW row_pointer[1];
jpeg_source_mgr mgr;

image_transport::Publisher pub; 

void init_source(j_decompress_ptr cinfo) {
	mgr.next_input_byte = (JOCTET*) src;
	mgr.bytes_in_buffer = size;
}

boolean fill_input_buffer(j_decompress_ptr cinfo) {
	std::cout << "Problem with incoming file" << std::endl;
	exit(-1);
}

void skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
	mgr.next_input_byte += num_bytes;
	mgr.bytes_in_buffer -= num_bytes;
}

void term_source(j_decompress_ptr cinfo) {
	return;
}

void vector_src(j_decompress_ptr cinfo) {
	mgr.init_source = &init_source;
	mgr.fill_input_buffer = &fill_input_buffer;
	mgr.skip_input_data = &skip_input_data;
	mgr.resync_to_restart = &jpeg_resync_to_restart;
	mgr.term_source = &term_source;
	cinfo->src = &mgr;
}

using namespace std;

int skip = 5;

void eyesCallback(const NaoImageTransport::NaoimageConstPtr& msg) {
	copy(msg->jpeg.begin(), msg->jpeg.end(), src);
	size = msg->jpeg.size();

	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);

	vector_src(&cinfo);

	jpeg_read_header(&cinfo, TRUE);	
	jpeg_start_decompress(&cinfo);

	int count = 0;
	while(count < cinfo.image_height) {
		row_pointer[0] = &image[count*WIDTH*CHANNELS];
		jpeg_read_scanlines(&cinfo, row_pointer, 1);
		count++;
	}

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

	IplImage* cvImage = cvCreateImage(cvSize(WIDTH,HEIGHT),IPL_DEPTH_8U,3);

	//RGB --> BGR
	for (int i = 0; i < WIDTH; i++) {
		for (int j = 0; j < HEIGHT; j++) {
			for (int c = 0; c < CHANNELS; c++) {
((uchar *)(cvImage->imageData + j*cvImage->widthStep))[i*cvImage->nChannels + c]= 
	image[j*WIDTH*CHANNELS + i*CHANNELS + CHANNELS-1-c];
			}
		}
	}

	sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(cvImage, "bgr8");
	pub.publish(out);

	cvReleaseImage(&cvImage);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nao_image_transport");
  ros::NodeHandle n;
  ros::Subscriber chatter_sub = n.subscribe("eyes", 1, eyesCallback);

  image_transport::ImageTransport it(n);
  pub = it.advertise("naocam/image", 1);

  ros::spin();
}

