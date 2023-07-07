/*
 * @Description:
 * @Version: 1.0
 * @Author: wyx
 * @Date: 2023-04-12 20:31:50
 * @LastEditors: wyx
 * @LastEditTime: 2023-04-28 21:22:48
 * @FilePath: /Wheel-legged_PC/Src/USB/commSerial.hpp
 * Copyright (C) 2023 wyx. All rights reserved.
 */
#ifndef COMMSERIAL_HPP
#define COMMSERIAL_HPP

#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <poll.h>
#include <User_inc.h>
using namespace std;
#define MAX_STR 255
/* ����֡С��256�ֽ� */
#define TX_DATA_LEN (48) // 14��uint8+10��float+6֡β     14+48+6
// #define RX_DATA_LEN (112 + 36 + 4 + 5)      // 4*7*4 + 9*4 + 4 + 5
#define RX_DATA_LEN (25)


void signal_handler_IO(int status);
// after it are the information you want to know
class commSerial
{
private:
    /* data */
   // struct termios termAttr;
    struct sigaction saio;
    int ifSerial = 0;
    const char *this_port;
    uint32_t this_baudrate;
    int handle;

public:
    commSerial(const char *port = "", uint32_t baudrate = B115200);
    ~commSerial();

    void commSerialInit();
    int commSerialRead();
    void commSerialWrite(char *data);
    void commSerialPrintResult();
    int commSerialIsopen();
    unsigned char txbuf[TX_DATA_LEN];
    unsigned char rxbuf[RX_DATA_LEN];

    int pps_Init(int &fd, char *port, int baudrate);
};

extern int PPS_handle;
#endif
