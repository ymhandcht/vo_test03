#include<ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>     //C99������������.h�ļ�
#include <stdarg.h>     //���ܿɱ����.h�ļ�
#include <ctype.h>      //C�����ַ����ຯ��,���ڲ����ַ��Ƿ������ض����ַ����
#include <fcntl.h>      //���ļ��Ĵ򿪡�����д�롢���ݶ�ȡ���ر��ļ��Ĳ���
#include <pthread.h>    //�߳��ļ�
#include <sys/socket.h> //C��TCP/IPЭ��ͨ���м�����
#include <fstream>      //C++�ļ���д.h�ļ�
#include <memory>       //make_shared    shared_ptr
/*Ubuntu.h*/
#include <net/if.h>    //����IP��ַ,����ӿ�
#include <unistd.h>    //�� POSIX ����ϵͳ API �ķ��ʹ��ܵ�ͷ�ļ�
#include <errno.h>     //Ubuntuϵͳ��������
#include <sys/time.h>  //linux����ʵ�ֶ�ʱ��.h�ļ�
#include <sys/sem.h>   //�ź���.h�ļ�      POSIX�����XSI��չͷ�ļ�
#include <sys/ioctl.h> //Ubuntuϵͳ��������ͷ�����ļ�
#include <semaphore.h> //�ź���.h�ļ�    POSIX����Ŀ�ѡͷ�ļ�
#include <linux/can.h> //can.h�ļ�
#include <termios.h>   //Ubuntu���ڱ��.h�ļ�
/*python.h*/
// #include <sched.h> // pyhthon��������
/*User.h*/
#include <eigen3/Eigen/Dense>
#include <vector>
/*#define interface*/
#define USB_COMM
#define UARTCOM
#define PPS_COMM
typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;
typedef typename Eigen::Vector2d Vec2d;

#define PI 3.1415926