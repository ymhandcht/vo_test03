#ifndef _COORDCACULATE_H_
#define _COORDCACULATE_H_

#include<ros/ros.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<deque>
#include<numeric>
#include<orb_features_cuda.h>
#include<mindVision_init.h>
#include<imu.h>

using namespace cv;
using namespace std;
#define downWidth  1280*0.75
#define downHeigth 1024*0.75
#define x_point downWidth/2
#define y_point downHeigth/2


#define realDis1  1 //125/390
#define realDis2  1 //125/390

// float calculateRevolveAngle(Point2f p1, Point2f p2, Point2f p3, Point2f p4);

float pointToLineDistance(float x1,float y1,float x2,float y2,float x3,float y3);

float getPointToPointDistance(float x1,float y1,float x2,float y2);

Point2f getCrossPoint(float k1,float b1,float k2,float b2);
Point2f getSolution(float a,float b,float c);
Point3f updateCoord(deque<Point3f> &d);
Point3f getNowCoord(Point3f last_point,Point3f new_point);

class CoordCaculate
{
public:
    IMU imu;
    vector<Point3f> vec1;  // 记录第一幅图像特征点容器
    vector<Point3f> vec2;  //记录匹配图像特征点容器

    vector<Point2f> vecc1;
    vector<Point2f> vecc2;
    Point3f mappedPoint;         // 映射的中心点坐标
    deque<Point3f> mappedPoints; //经过筛选后得到的所有映射中心点的坐标
    Point3f filterPoints;//经过滤波过后的映射点坐标
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;
    float sumAngles = 0.0;

    float imuAngle;  //通过imu获取到的角度信息
    int handle;

    float revovleAngle;
    deque<float> revolveAngles;  //通过至多50个特征点算出的姿态角度变化数组
    float angleResult;  //经过滤波得到的相机计算角度结果

    //将特征点存放到vec1 vec2两个容器内
    void storageFeaturePoints(vector<DMatch> &matches,vector<KeyPoint> &keyPoints_1,vector<KeyPoint> &keyPoints_2);
    //计算匹配图像对应第一幅图像中心点的映射点  输入为两个位置 且这两个点不是同一个点
    void getMappedPoint(int firstPoint,int secondPoint);
    //筛选得到所有映射中心点坐标
    void getAllMappedPoints();
    //对所有的坐标进行滤波处理
    void filterMappedPoints();
    //初始化
    void initCoordCaculate();

    //计算一对特征点得到相机旋转角度
    void calculateRevolveAngle(int firstPoint,int secondPoint);
    //筛选得到所有的旋转角度 并将计算出的角度存入 deque<float>revolveAngles数组
    void getAllRevolveAngles();
    //对所有角度滤波处理 得到angleResult
    void filterAngles();
    //存储imu获得的角度 航向角
    void storageImuAngle();
};
#endif