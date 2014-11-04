#ifndef _INEMO_M1_DRIVER_H_
#define _INEMO_M1_DRIVER_H_

#include <QThread>
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

using namespace std;

namespace inemo
{

class CInemoDriver : public QThread
{
    Q_OBJECT

public:
    CInemoDriver();
    ~CInemoDriver();

    bool startIMU();
    bool stopIMU();

protected:
    // >>>>> iNemo Protocol
    // <<<<< iNemo Protocol

    /*!
     * \brief run
     * Process the serial data continously when
     * the \ref iNEMO_Start_Acquisition is called
     *
     */
    void run() Q_DECL_OVERRIDE;
    void processSerialData( string& data );

private:
    ros::NodeHandle m_nh;

    serial::Serial mSerial;

    string mSerialPort;
    uint32_t mBaudrate;
    uint32_t mTimeout;

    bool mStopped; ///< Used to stop the serial processing

    uint64_t mNoDataCounter;
};

}

#endif

