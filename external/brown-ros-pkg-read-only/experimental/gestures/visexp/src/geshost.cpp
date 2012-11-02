#include <iostream>
#include <opencv/cv.h>

#include "ges.hpp"

int main(int argc, char** argv) {
	IplImage* empty = cvCreateImage(cvSize(500,312), IPL_DEPTH_8U, 1);

	person_t person;
	processFrame(empty, 500, 312, .1, 4, 50, &person);

	cvReleaseImage(&empty);
	return 0;
}
