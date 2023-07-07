#ifndef _CameraParameter_H
#define _CameraParameter_H

#include<ros/ros.h>
#include<iostream>
#include<mindVision_init.h>

using namespace std;

class Camera
{
public:
//相机内参
    static float camera_fx;
    static float camera_fy;
    static float camera_cx;
    static float camera_cy;
//相机畸变参数
    
    static float camera_k1;
    static float camera_k2;
    static float camera_k3;
    static float camera_p1;
    static float camera_p2;
};

#endif