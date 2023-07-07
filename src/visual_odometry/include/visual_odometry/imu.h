#ifndef _IMU_H_
#define _IMU_H_
#include<ros/ros.h>
#include <termios.h>
#include<vector>
#include<User_inc.h>

using namespace std;
#define RX_PPS_DATA_LEN (36)


class IMU
{
    private:
        char header1, header2, end1, end2;//数据传输帧头帧尾
        
        struct termios termAttr;
       

    public:
        IMU();
        
        vector<float> dataResult;  //存储三轴角度 角速度 和三轴加速度

        int imu_init(int &fd, char *port, int baudrate); // 初始化数据接收端口和数据传输波特率
        void readIMUdata(int fd);  //读取IMU数据
        void storageData(char *data);  //将IMU数据转化成float数据
        
};
#endif