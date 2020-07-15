#include "slamBase.h"


void print_matInfo(cv::Mat img) {
	string mat_type;
	if (img.depth() == CV_8U) mat_type = "CV_8U";
	else if (img.depth() == CV_8S) mat_type = "CV_8S";
	else if (img.depth() == CV_16U) mat_type = "CV_16U";
	else if (img.depth() == CV_16S) mat_type = "CV_16S";
	else if (img.depth() == CV_32S) mat_type = "CV_32S";
	else if (img.depth() == CV_32F) mat_type = "CV_32F";
	else if (img.depth() == CV_64F) mat_type = "CV_64F";

	cout << mat_type << endl;
}

int main(void)
{
    cv::Mat img = cv::imread("/home/park/사진/depthimg.png");

    print_matInfo(img);
    cv::imshow("depth", img);
    cv::waitKey(0);
    
    return 0;
}