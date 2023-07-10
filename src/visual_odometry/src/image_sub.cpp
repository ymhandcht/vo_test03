#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include <opencv2/imgproc/imgproc.hpp>

#include<ros/ros.h>
#include<orb_features.hpp>
#include<coordCaculate.h>
#include<iostream>
#include<orb_features_cuda.h>

#include "orb_extractor1.h"
#include "vfc.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>


using namespace cv;
using namespace std;
using namespace XIAOC;

static Point3f realCoor(0.0,0.0,0.0);
Point3f lastcastCoord(downWidth/2, downHeigth/2, 0);//图像坐标中心点


static Mat image1,image2;
static int i = 0;
static int num = 0;
#define Z 0.46

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    
    if(i!=0)
    {
    //定义初始坐标为（0，0，0）
    Mat img;
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    img = cv_ptr->image;//opencv类型图像，这一行等价于cv_ptr->image.copyTo(img);

    image1 = img;  //首先记录第一张图像
        /*以下是cuda版本测试*/

        float mStart = (float)getTickCount();
        orb_cuda_features ocf(image1,image2);
        ocf.orb_cuda_features_matching();
        CoordCaculate cc;
        cc.initCoordCaculate();
        //将匹配的特征点放到容器里 分别存放第一幅和第二幅图像匹配的特征点
        cc.storageFeaturePoints(ocf.matches,ocf.keyPoints_1,ocf.keyPoints_2);
        cc.getAllMappedPoints();
        deque<Point3f> result = cc.mappedPoints;
        
        cc.filterMappedPoints();
        cout << "vec1.size = " << cc.vec1.size() << endl;

        Point3f coord = getNowCoord(lastcastCoord,cc.filterPoints);
        realCoor+=coord;  //得到真实坐标
        
        cout<<"当前坐标为："<<realCoor<<endl;
        
        image2 = image1;

        num++;
        cout<<"num = "<<num<<endl;
    }
    else
    {
    Mat img;
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    img = cv_ptr->image;//opencv类型图像，这一行等价于cv_ptr->image.copyTo(img);
    image1 = img;  //首先记录第一张图像
    image2 = image1;
    i++;
    }
        
    

}

 static Eigen::Matrix<float, 3, 3> R_result = Eigen::Matrix3f::Identity();
 static Eigen::Matrix<float, 3, 1> t_result = Eigen::Matrix<float, 3, 1>::Zero();

void imageCallback1(const sensor_msgs::ImageConstPtr &msg)
{
    Mat K_Matrix = (Mat_<float>(3, 3) << 614.7144, 0, 322.7901, 0, 614.6425, 245.56085, 0, 0, 1);
    Mat distMatrix = (Mat_<float>(1, 5) << 0, 0, 0, 0, 0);

    int nfeatures = 1000;
    int nlevels = 1;
    float fscaleFactor = 1.0;
    float fIniThFAST = 40;
    float fMinThFAST = 10;
    if(i!=0)
    {
        Mat img;
        cv_bridge::CvImagePtr cv_ptr; 
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        img = cv_ptr->image;//opencv类型图像，这一行等价于cv_ptr->image.copyTo(img);
        image1 = img;
        Mat grayImgsrc, mask;
        cv::cvtColor( image1, grayImgsrc, 6 );
        Mat grayImgcur,maskcur;
        cv::cvtColor( image2, grayImgcur, 6 );
        ORBextractor *pORBextractor;
        pORBextractor = new ORBextractor( nfeatures, fscaleFactor, nlevels, fIniThFAST, fMinThFAST );
        Mat srcdesc;
        vector<KeyPoint> srckps;
        (*pORBextractor)( grayImgsrc, mask, srckps, srcdesc );
        Mat curdesc;
        vector<KeyPoint> curkps;
        (*pORBextractor)( grayImgcur, maskcur, curkps, curdesc );
        BFMatcher matcher_bf(NORM_HAMMING, true); //使用汉明距离度量二进制描述子，允许交叉验证
        vector<DMatch> Matches_bf;
        matcher_bf.match(srcdesc, curdesc, Matches_bf);
        if(Matches_bf.size() == 0)
            cout << "no match" << endl;

        vector<Point2f> X;
        vector<Point2f> Y;
        X.clear();
        Y.clear();

        for(int i=0;i<Matches_bf.size();i++){
            int index1 = Matches_bf.at(i).queryIdx;
            int index2 = Matches_bf.at(i).trainIdx;
            X.push_back(srckps.at(index1).pt);
            Y.push_back(curkps.at(index2).pt);
        }

        VFC myvfc;
        myvfc.setData(X, Y);
        myvfc.optimize();
        vector<int> matchIdx = myvfc.obtainCorrectMatch();

        // 筛选正确的匹配
        std::vector< DMatch > Matches_VFC;
        for (unsigned int i = 0; i < matchIdx.size(); i++) {
            int idx = matchIdx[i];
            Matches_VFC.push_back(Matches_bf[idx]);
        }
        //第一幅图像地图点
        vector<cv::Point3f> vSrc_mappoints ;
        
        vector<cv::Point2f> vCur;
        for (int i = 0; i < Matches_VFC.size(); ++i){
                int index1 = Matches_VFC[i].queryIdx;
                int index2 = Matches_VFC[i].trainIdx;
                float x1 = srckps[index1].pt.x;
                float y1 = srckps[index1].pt.y;
            
                float z1 = Z;
                float x = z1 * (x1 - 322.790100) / 614.7144165;
                float y = z1 * (y1 - 245.560852) / 614.642517089;
                float z = z1;
            
                cv::Point3f v1(x, y, z);
                float x2 = curkps[index2].pt.x;
                float y2 = curkps[index2].pt.y;
                cv::Point2f v2(x2, y2);
                vCur.emplace_back(v2);
                vSrc_mappoints.push_back(v1);
       
            // cout << vSrc_mappoints[i] << endl;
        }
            //第二幅图像像素点
            Mat R,t;
            cv::solvePnPRansac(vSrc_mappoints, vCur, K_Matrix, distMatrix, R, t);
            Mat Rvec;
            Mat_<float> Tvec;
            R.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
            t.convertTo(Tvec, CV_32F); // 平移向量转换格式 

            Mat_<float> rotMat(3, 3);
            Rodrigues(Rvec, rotMat);
            // 旋转向量转成旋转矩阵
            // cout << "rotMat" << endl << rotMat << endl << endl;
            // cout << t << endl;
        

        // Mat P_oc;
        // P_oc = -rotMat.inv() * Tvec;
        // // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
        // cout << "P_oc" << endl << P_oc << endl;
        Eigen::Matrix<float, 3, 3> m1;
        Eigen::Matrix<float, 3, 1> m2;
        // cout << "R = " << R << endl;
        //cout << "t = " << Tvec.at<float>(0,0) << endl;

        for (int i = 0; i < 3; i++)
        {
            m2(i, 0) = Tvec.at<float>(i, 0);

            for (int j = 0; j < 3; j++)
            {
                m1(i, j) = rotMat.at<float>(i, j);
            }
        }
        R_result = m1*R_result;
        t_result += m2;

        image2 = image1;

        cout << "R_result = " << R_result << endl;
        cout << "t_result = " << t_result << endl;
    }
    else
    {
        Mat img;
        cv_bridge::CvImagePtr cv_ptr; 
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        img = cv_ptr->image;//opencv类型图像，这一行等价于cv_ptr->image.copyTo(img);
        image1 = img;  //首先记录第一张图像
        image2 = image1;
        i++;
    }

}

int main(int argc,char * argv[])
{
    ros::init(argc,argv,"image_sub");

    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw",100,imageCallback1);
    ros::spin();

    return 0;
}