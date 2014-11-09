#include <inemo_m1_driver.h>

#include <QMutexLocker>

namespace inemo
{

CInemoDriver::CInemoDriver() :
    QThread( NULL )
{
    // TODO: add parameters for serial port initialization!!!

    mSerialPort = "/dev/ttyUSB0";
    mBaudrate = 56000;
    mTimeout = 100;

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

    // TODO call iNEMO_Start_Acquisition that "start"s the thread if serial reply is ACK

    return true;
}

bool CInemoDriver::stopIMU()
{
    mStopped = true;
}

bool CInemoDriver::pauseIMU( bool paused )
{
    QMutexLocker locker(mMutex);

    mPaused = paused;

    if(mPaused)
    {
        // TODO call iNEMO_Stop_Acquisition to stop data receiving
    }
    else
    {
         // TODO call iNEMO_Start_Acquisition to restart the data reception
    }
}

void CInemoDriver::run()
{
    mStopped = false;
    mPaused = false;
    mNoDataCounter = 0;

    ROS_INFO_STREAM( "IMU Data acquiring loop started");

    while(!mStopped)
    {
        ros::spinOnce(); // Processing ROS messages

        if(!mPaused)
        {
            if( mSerial.available() )
            {
                ROS_DEBUG_STREAM("Reading from serial port");

                string data = mSerial.read(mSerial.available());

                ROS_DEBUG_STREAM("Read: " << data);

                processSerialData( data );
            }
            else
            {
                mNoDataCounter++;
                ROS_INFO_STREAM( "No IMU data since more than " << mNoDataCounter * mTimeout << " msec" );
            }
        }
        else
            msleep( 10 );
    }

    ROS_INFO_STREAM( "IMU Data acquring loop stopped");
}

void CInemoDriver::processSerialData( string& data )
{
    // TODO Process iNEMO_Acquisition_Data message
    // to extract attitude and quaternion
}

}


