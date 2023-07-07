#ifndef _orb_features_cuda_H
#define _orb_features_cuda_H


#include<ros/ros.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/xfeatures2d.hpp>
#include<mindVision_init.h>

#include "opencv2/cudabgsegm.hpp"
#include "opencv2/core/cuda.hpp"
#include"opencv2/core/cuda_stream_accessor.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"
#include"cuda.h"

//#include<opencv2/core/eigen.hpp>
#include<eigen3/Eigen/Dense>
#include<CameraParameter.h>

using namespace cv;
using namespace std;
using namespace cuda;

void knnMatches2(Mat &query, Mat &train, vector<DMatch> &matches);
void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u);
void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P);

class orb_cuda_features
{
public:
    orb_cuda_features(Mat img1,Mat img2);  //构造函数声明
    void orb_cuda_features_matching();          //特征提取
    
    Mat orignalImage_1; //待匹配图像
    Mat orignalImage_2;

    cuda::GpuMat G_img1, G_img2;       // gpu格式的图像数据
    cuda::GpuMat G_imggray1,G_imggray2;//将图像转化成灰度图像
    cuda::GpuMat G_keypoints1, G_keypoints2; //GPU格式两幅图像的特征点
	cuda::GpuMat G_descriptors1, G_descriptors2, G_descriptors1_32F, G_descriptors2_32F;  //描述子
	vector<KeyPoint> keyPoints_1, keyPoints_2;
	//Mat descriptors1,descriptor2;
	vector<DMatch> matches;
    Mat matchImage;

};
#endif