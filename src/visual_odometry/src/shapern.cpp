#include <iostream>
#include<ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;


Mat Sharpen(Mat input, int percent, int type)
{
	Mat result;
	Mat s = input.clone();
	Mat kernel;
	switch (type)
	{
	case 0:
		kernel = (Mat_<int>(3, 3) <<
			0, -1, 0,
			-1, 4, -1,
			0, -1, 0
			);
	case 1:
		kernel = (Mat_<int>(3, 3) <<
			-1, -1, -1,
			-1, 8, -1,
			-1, -1, -1
			);
	default:
		kernel = (Mat_<int>(3, 3) <<
			0, -1, 0,
			-1, 4, -1,
			0, -1, 0
			);
	}
	filter2D(s, s, s.depth(), kernel);
	result = input + s * 0.01 * percent;
	return result;
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"sharpen");
	Mat scr = imread("/home/action/vo_test03/src/picture1/110个数：2.164158序号174.jpg");
	Mat res = Sharpen(scr, 100, 0);
	imwrite("/home/action/vo_test03/src/picture1/dog.jpg",res);
	imshow("原图", scr);
	imshow("处理后", res);
	waitKey(0);
	return 0;
}
