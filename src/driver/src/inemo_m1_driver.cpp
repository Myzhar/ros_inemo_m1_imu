#include <inemo_m1_driver.h>

namespace inemo
{

CInemoDriver::CInemoDriver()
{
    // TODO: add parameters for serial port initialization!!!

    mSerialPort = "/dev/ttyUSB0";
    mBaudrate = 56000;
    mTimeout = 500;

}

CInemoDriver::~CInemoDriver()
{

}

bool CInemoDriver::startIMU()
{
    // TODO: replace with serial parameters
    try
    {
        mSerial.setPort(mSerialPort);
        mSerial.setBaudrate(mBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(mTimeout);

        mSerial.setTimeout(to);
        mSerial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << mSerialPort << " - Error: "  << e.what() );
        return false;
    }

    if(mSerial.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized: " << mSerialPort );
    }
    else
    {
        ROS_ERROR_STREAM( "Serial port not opened: " << mSerialPort );
        return false;
    }

    // TODO start timers!

    return true;
}

}


