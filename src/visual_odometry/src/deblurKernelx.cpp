#include<ros/ros.h>
#include<opencv2/core/core.hpp>
#include<opencv2/core/core_c.h>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;

enum ConvolutionType                     // 函数 conv2 卷积时参数的类型
{
    CONVOLUTION_FULL,                    // 卷积时的参数，和 matlab 的 full 一致
    CONVOLUTION_SAME,                    // 卷积时的参数，和 matlab 的 same 一致
    CONVOLUTION_VALID                    // 卷积时的参数，和 matlab 的 valid 一致
};

Mat deblur_DFTi(const Mat& blurimg, const Mat& kernel, ConvolutionType type, int ddepth)
{
    Mat convRes;
    int dft_M = getOptimalDFTSize(blurimg.rows + kernel.rows - 1);   // 行数
    int dft_N = getOptimalDFTSize(blurimg.cols + kernel.cols - 1);   // 列数

    Mat imagePad(dft_M, dft_N, CV_32FC1, Scalar(0));
    Mat imagePadROI = imagePad(Rect(0, 0, blurimg.cols, blurimg.rows));
    blurimg.convertTo(imagePadROI, CV_32FC1, 1, 0);
//要分别用到实部和虚部计算，所以都要保留
    Mat planes[] = { Mat_<float>(imagePad), Mat::zeros(imagePad.size(),CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);

    Mat kernelPad(dft_M, dft_N, CV_32FC1, Scalar(0));
    Mat kernelPadROI = kernelPad(Rect(0, 0, kernel.cols, kernel.rows));
    kernel.convertTo(kernelPadROI, CV_32FC1, 1, 0);
    Mat planesk[] = { Mat_<float>(kernelPad), Mat::zeros(kernelPad.size(),CV_32F) };
    Mat complexk;
    merge(planesk, 2, complexk);

    dft(complexI, complexI);
    dft(complexk, complexk);
    split(complexI, planes);
    split(complexk, planesk);
    Mat mulout[2];
//和上面的公式对应的操作
    Mat sub = planesk[0].mul(planesk[0]) + planesk[1].mul(planesk[1]);
    mulout[0] = planes[0].mul(planesk[0]) + planes[1].mul(planesk[1]);
    divide(mulout[0], sub, mulout[0]);//divide(I1,I2,dst,scale,int dtype=-1);
    mulout[1] = planesk[0].mul(planes[1]) - planesk[1].mul(planes[0]);
    divide(mulout[1], sub, mulout[1]);
    Mat out;
    merge(mulout, 2, out);
    idft(out, out, DFT_REAL_OUTPUT);
    normalize(out, out, 0, 1, CV_MINMAX);
    out.convertTo(out, CV_8UC1, 255);

    Rect r;
    switch(type)
    {
    case CONVOLUTION_FULL:  // full
        r = Rect(0, 0, blurimg.cols + kernel.cols - 1, blurimg.rows + kernel.rows - 1);
        break;
    case CONVOLUTION_SAME:  // same
        r = Rect((kernel.cols + 0.5) / 2, (kernel.rows + 0.5) / 2, blurimg.cols, blurimg.rows);
        break;
    case CONVOLUTION_VALID:  // valid
        r = Rect((kernel.cols + 0.5) / 2, (kernel.rows + 0.5) / 2, blurimg.cols - kernel.cols + 1, blurimg.rows - kernel.rows + 1);
        break;
    default:  // same
        r = Rect((kernel.cols + 0.5) / 2, (kernel.rows + 0.5) / 2, blurimg.cols, blurimg.rows);
        break;
    }

    out(r).convertTo(convRes, CV_8UC1, 1, 0);
    return convRes;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"main_test");
    Mat orignalImage = imread("/home/action/vo_test03/src/picture/3---nan.jpg");
    
    //deblur_DFTi(orignalImage,)
    Mat outimgdft = orignalImage.clone();
    int wk = 24; //卷积核长度
   Mat kernelx = Mat::ones(1, wk, CV_32FC1) / wk;
    
   // Mat kernelx = (Mat_<float>(1,10)<<(1, 0, 1, 0, 1, 1, 0, 1, 0, 0) / 5);
    Mat result = deblur_DFTi(outimgdft,kernelx,CONVOLUTION_VALID, 0);
    imshow("src1",orignalImage);
    imshow("src2",result);
    waitKey(0);

    return 0;
}