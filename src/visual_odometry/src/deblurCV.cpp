/**
* @brief You will learn how to recover an image with motion blur distortion using a Wiener filter
* 您将学习如何使用维纳滤波器恢复具有运动模糊失真的图像
* @author Karpushin Vladislav, karpushin@ngs.ru, https://github.com/VladKarpushin
*/

#include<deblurCV.h>
using namespace cv;
using namespace std;
 

 
const String keys =
"{help h usage ? |             | print this message				}"
"{image          |/home/action/vo_test03/src/picture/57个数：4.466268序号53.jpg    | input image name				}"
"{LEN            |3      | length of a motion				}"
"{THETA          |5          | angle of a motion in degrees	}"
"{SNR            |2000         | signal to noise ratio信噪比	}"
;
// 运动模糊图像恢复算法    包括 PSF 生成、Wiener 滤波器生成和在频域中过滤模糊图像：
int main(int argc, char *argv[])
{
   
    CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
 
    int LEN = parser.get<int>("LEN");
    double THETA = parser.get<double>("THETA");
    int snr = parser.get<int>("SNR");
    string strInFileName = parser.get<String>("image");
 
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }
 
    Mat imgIn;
    imgIn = imread(strInFileName, IMREAD_GRAYSCALE);
    Mat imgCorecct = imread("/home/action/vo_test03/src/picture/57个数：4.466268序号53.jpg",IMREAD_GRAYSCALE);
    if (imgIn.empty()) //检查图像是否加载 
    {
        cout << "ERROR : Image cannot be loaded..!!" << endl;
        return -1;
    }
 
    Mat imgOut;
 
//! [main]
    // it needs to process even image only 图像的大小为 2*2,4*4 ，8*8 ………………都是可以进行去模糊的
    Rect roi = Rect(0, 0, imgIn.cols & -2, imgIn.rows & -2);//
 
    //Hw calculation (start)
    Mat Hw, h;
    calcPSF(h, roi.size(), LEN, THETA);
    calcWnrFilter(h, Hw, 1.0 / double(snr));
    //Hw calculation (stop)
 
    imgIn.convertTo(imgIn, CV_32F);
    //edgetaper(imgIn, imgIn);//边缘逐渐变细
 
    // filtering (start)
    filter2DFreq(imgIn(roi), imgOut, Hw);//频域进行滤波
    // filtering (stop)
//! [main]
 
    imgOut.convertTo(imgOut, CV_8U);
    normalize(imgOut, imgOut, 0, 255, NORM_MINMAX);
    //imwrite("/home/action/vo_test03/src/picture/result5+2+1500.jpg", imgOut);
    imshow("src",imgOut);
    imshow("对比",imgCorecct);
    waitKey(0);
    return 0;
}

 
//! [calcPSF] 根据输入参数 LEN 和 THETA（以度为单位）形成一个 PSF     线性运动模糊失真的点扩散函数（PSF）是一条线段
void calcPSF(Mat& outputImg, Size filterSize, int len, double theta)
{
    Mat h(filterSize, CV_32F, Scalar(0));
    Point point(filterSize.width / 2, filterSize.height / 2);//中心点  椭圆圆心
    ellipse(h, point, Size(0, cvRound(float(len) / 2.0)), 90.0 - theta, 0, 360, Scalar(255), FILLED);//椭圆
    Scalar summa = sum(h);
    outputImg = h / summa[0];//归一化
}
//! [calcPSF]
 
//! [fftshift] fft变换后进行频谱搬移
void fftshift(const Mat& inputImg, Mat& outputImg)
{
    outputImg = inputImg.clone();
    int cx = outputImg.cols / 2;
    int cy = outputImg.rows / 2;
    Mat q0(outputImg, Rect(0, 0, cx, cy));
    Mat q1(outputImg, Rect(cx, 0, cx, cy));
    Mat q2(outputImg, Rect(0, cy, cx, cy));
    Mat q3(outputImg, Rect(cx, cy, cx, cy));
    Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}
//! [fftshift]
 
//! [filter2DFreq] 在频域中过滤模糊图像，再转换到时域
void filter2DFreq(const Mat& inputImg, Mat& outputImg, const Mat& H)
{
    Mat planes[2] = { Mat_<float>(inputImg.clone()), Mat::zeros(inputImg.size(), CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);
    dft(complexI, complexI, DFT_SCALE);
 
    Mat planesH[2] = { Mat_<float>(H.clone()), Mat::zeros(H.size(), CV_32F) };
    Mat complexH;
    merge(planesH, 2, complexH);
    Mat complexIH;
    mulSpectrums(complexI, complexH, complexIH, 0);//频域相乘进行滤波
 
    idft(complexIH, complexIH);//变换到时域
    split(complexIH, planes);
    outputImg = planes[0];//时域输出滤波后图像
}
//! [filter2DFreq]
 
//! [calcWnrFilter] 计算维纳滤波器
void calcWnrFilter(const Mat& input_h_PSF, Mat& output_G, double nsr)
{
    Mat h_PSF_shifted;
    fftshift(input_h_PSF, h_PSF_shifted);//点扩散函数 先进行搬移
    Mat planes[2] = { Mat_<float>(h_PSF_shifted.clone()), Mat::zeros(h_PSF_shifted.size(), CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);
    dft(complexI, complexI);//傅里叶变换
    split(complexI, planes);
    Mat denom;
    pow(abs(planes[0]), 2, denom);//|H|^2
    denom += nsr;//|H|^2+1/snr
    divide(planes[0], denom, output_G);//H/(|H|^2+1/snr)  ->output_G
}
//! [calcWnrFilter]
 
//! [edgetaper]使输入图像的边缘逐渐变细，以减少恢复图像中的振铃效应
//通过edgetaper在反卷积之前调用该函数，可以减少噪声放大和沿图像边界的振铃效应。 图像恢复对噪声功率参数的敏感性降低
void edgetaper(const Mat& inputImg, Mat& outputImg, double gamma, double beta)//对图像边缘进行模糊处理
{
    int Nx = inputImg.cols;
    int Ny = inputImg.rows;
    Mat w1(1, Nx, CV_32F, Scalar(0));
    Mat w2(Ny, 1, CV_32F, Scalar(0));
 
    float* p1 = w1.ptr<float>(0);
    float* p2 = w2.ptr<float>(0);
    float dx = float(2.0 * CV_PI / Nx);
    float x = float(-CV_PI);
    for (int i = 0; i < Nx; i++)
    {
        p1[i] = float(0.5 * (tanh((x + gamma / 2) / beta) - tanh((x - gamma / 2) / beta)));
        x += dx;
    }
    float dy = float(2.0 * CV_PI / Ny);
    float y = float(-CV_PI);
    for (int i = 0; i < Ny; i++)
    {
        p2[i] = float(0.5 * (tanh((y + gamma / 2) / beta) - tanh((y - gamma / 2) / beta)));
        y += dy;
    }
    Mat w = w2 * w1;
    multiply(inputImg, w, outputImg);
}
//! [edgetaper]
