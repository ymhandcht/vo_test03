#include<imu.h>
#include<iostream>
#include<ros/ros.h>
#include <fcntl.h>
#include <termios.h>
#include<commSerial.h>

using namespace std;

IMU::IMU()
{

}


int IMU::imu_init(int &fd,char *port, int baudrate)
{
    cout << "imu init begin ......" << endl;
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("open_port: Unable to open\n");
        return 0;
    }
    tcgetattr(fd, &termAttr);
    bzero(&termAttr, sizeof(termAttr));
    cfsetispeed(&termAttr, baudrate);
    cfsetospeed(&termAttr, baudrate);
    termAttr.c_cflag &= ~PARENB;
    termAttr.c_cflag &= ~CSTOPB;
    termAttr.c_cflag &= ~CSIZE;
    termAttr.c_cflag |= CS8;
    termAttr.c_cflag |= (CLOCAL | CREAD);
    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | ISTRIP);
    termAttr.c_oflag &= ~OPOST;
    termAttr.c_cc[VMIN] = 1;
    termAttr.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &termAttr);
    tcflush(fd, TCIOFLUSH);
    std::cout << port << ": serial init ok!" << endl;
    return fd;
}

void IMU::readIMUdata(int fd)
{
    int pps_len;
    int count = 0;
    int dataCnt = 0;
    char ch;
    union
    {
        char dataC[RX_PPS_DATA_LEN];
        float dataF[RX_PPS_DATA_LEN / 4];
    } ppsData;
    while (fd)
    {
       
        pps_len = read(fd, &ch, 1);
        
        if (pps_len > 0)
        {
            //cout << "pps_len = " << pps_len << endl;
            switch (count)
            {
            case 0:
                if (ch == 'A')
                {
                    count++;
                }
                else
                    count = 0;
                break;
            case 1:
                if (ch == 'T')
                {
                    dataCnt = 0;
                    count++;
                }
                else
                    count = 0;
                break;
            case 2:
                ppsData.dataC[dataCnt] = ch;
                dataCnt++;
                if (dataCnt >= RX_PPS_DATA_LEN)
                {
                    dataCnt = 0;
                    count++;
                }
                break;
            case 3:
                if (ch == 't')
                    count++;
                else
                    count = 0;
                break;
            case 4:
                count = 0;
                if (ch == 'a') //读取了一组数据
                {
                    //将数据转化成float再存入dataResult
                    this->storageData(ppsData.dataC);
                    return;
                    // cout << this->dataResult[2] << endl;
                }
                else
                    count = 0;
                break;
            default:
                break;
            }
        }
    }
}

void IMU::storageData(char *data)
{
    //首先清空数组原有数据
    this->dataResult.clear();
    union trans_t
    {
        float dataf[9];  //dataf[i]就是想要的float数据
        char datac[9 * 4];
    } trans;

    for (int i = 0; i < 9 * 4; ++i)
    {
        trans.datac[i] = data[i];
    }
    for (int i = 0; i < 9;i++)
       {
        this->dataResult.push_back(trans.dataf[i]); //将数据存入该数组
       }
       //遍历数组
       
}