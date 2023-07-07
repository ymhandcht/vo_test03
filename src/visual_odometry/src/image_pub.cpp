#include<ros/ros.h>
#include<mindVision_init.h>
#include<CameraApi.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"image_pub");
    MindVisionInit mindvision;
    mindvision.init();

    ros::NodeHandle node;
    image_transport::ImageTransport transport(node);
    image_transport::Publisher image_pub = transport.advertise("imageTopic",100);
    ros::Time time = ros::Time::now();

    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";

    ros::Rate rate(15);
    while(ros::ok())
    {
        mindvision.updateImage();
        cvi.image = mindvision.dstImage;
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        image_pub.publish(im);

        cout<<"数据已经发布！"<<endl;
        mindvision.releaseBuffer();
        rate.sleep();
    }
    return 0;
}