#include<imu.h>

int handle;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "imu_test");
    IMU imu;
    handle = imu.imu_init(handle,"/dev/pps_uart",B115200);
    //cout << "handle = " << handle << endl;
   
    imu.readIMUdata(handle);
    cout << imu.dataResult[2] << endl;

    return 0;
}