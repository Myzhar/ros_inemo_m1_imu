#ifndef _INEMO_M1_DRIVER_H_
#define _INEMO_M1_DRIVER_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

using namespace std;

namespace inemo
{

class CInemoDriver
{

public:
    CInemoDriver();
    ~CInemoDriver();

    bool startIMU();

private:
    ros::NodeHandle m_nh;

    serial::Serial mSerial;

    string mSerialPort;
    uint32_t mBaudrate;
    uint32_t mTimeout;
}

}

#endif

