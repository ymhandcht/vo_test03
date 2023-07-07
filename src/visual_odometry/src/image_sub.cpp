#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>

#include<ros/ros.h>
#include<orb_features.hpp>
#include<coordCaculate.h>
#include<iostream>
#include<orb_features_cuda.h>

using namespace cv;
using namespace std;
static Point3f realCoor(0.0,0.0,0.0);
Point3f lastcastCoord(downWidth/2, downHeigth/2, 0);//图像坐标中心点


static Mat image1,image2;
static int i = 0;
static int num = 0;

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

int main(int argc,char * argv[])
{
    ros::init(argc,argv,"image_sub");

    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("imageTopic",100,imageCallback);
    ros::spin();

    return 0;
}