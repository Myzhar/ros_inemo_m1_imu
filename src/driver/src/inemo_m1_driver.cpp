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

    ROS_INFO_STREAM( "Starting data acquisition" );

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

    // TODO Configurate IMU before starting acquisition

    return iNEMO_Start_Acquisition();
}

bool CInemoDriver::stopIMU()
{
    mStopped = true;
}

bool CInemoDriver::pauseIMU( bool paused )
{
    QMutexLocker locker(&mMutex);

    bool reply;

    if(mPaused)
    {
        // TODO call iNEMO_Stop_Acquisition to stop data receiving
        reply = iNEMO_Stop_Acquisition();
    }
    else
    {
         // TODO call iNEMO_Start_Acquisition to restart the data reception
        reply = iNEMO_Start_Acquisition();
    }

    mPaused = paused;

    return mPaused;
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
            //if( mSerial.available() )
            if( mSerial.waitReadable() )
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

bool CInemoDriver::processSerialData( string& data )
{
    // TODO Process iNEMO_Acquisition_Data message
    // to extract attitude and quaternion
}

bool CInemoDriver::sendSerialCmd( quint8 frameControl, quint8 lenght, quint8 messId, QByteArray& payload )
{
    int dataSize = payload.size();

    if( lenght != dataSize+1 )
    {
        ROS_ERROR_STREAM( "Message lenght is wrong. Espected " << dataSize+1 << " received " << lenght );
        return false;
    }

    mSerialBuf[0] = frameControl;
    mSerialBuf[1] = lenght;
    mSerialBuf[2] = messId;

    if(payload.size()>0)
        memcpy( &mSerialBuf[3], payload.data(), payload.size() );

    int written = mSerial.write( mSerialBuf, dataSize+3 );
    if ( written != dataSize+3 )
    {
        ROS_ERROR_STREAM( "Serial write error. Written " << written << " bytes instead of" << dataSize+3 << " bytes.");
        return false;
    }

    return true;
}

bool CInemoDriver::iNEMO_Start_Acquisition()
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Start_Acquisition' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_INFO_STREAM( "Cannot start acquisition, serial port non opened");
        return false;
    }

    if( !mStopped && !mPaused )
    {
        ROS_ERROR_STREAM( "IMU acquisition must be stopped or paused before starting it again");
        return false;
    }

    quint8 frameControl = 0x20;
    quint8 lenght = 0x01;
    quint8 messId = 0x52;
    QByteArray payload; // No data!

    if( sendSerialCmd( frameControl, lenght, messId, payload ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout starting acquisition");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        if( reply.size() < 3 )
        {
            ROS_ERROR_STREAM( "Received incomplete reply starting acquisition");
            return false;
        }

        frameControl = reply.data()[0];
        lenght = reply.data()[1];
        messId = reply.data()[2];

        if( frameControl == 0x80 && lenght==0x01 && messId == 0x52 )
        {
            ROS_INFO_STREAM( "Received ACK: IMU acquisition started");

            // Start acquisition thread
            start();

            return true;
        }

        if( frameControl == 0xC0 && lenght==0x02 && messId == 0x52 )
        {
            ROS_ERROR_STREAM( "Received NACK: IMU acquisition not started. Error code: " << reply.data()[3] );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame" );
        return false;
    }

    return false;
}

bool CInemoDriver::iNEMO_Stop_Acquisition()
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Start_Acquisition' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_INFO_STREAM( "Cannot stop acquisition, serial port non opened");
        return false;
    }

    if( !mStopped && !mPaused )
    {
        ROS_ERROR_STREAM( "IMU acquisition is stopped, cannot stop it again");
        return false;
    }

    quint8 frameControl = 0x20;
    quint8 lenght = 0x01;
    quint8 messId = 0x53;
    QByteArray payload; // No data!

    if( sendSerialCmd( frameControl, lenght, messId, payload ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout stopping acquisition");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        if( reply.size() < 3 )
        {
            ROS_ERROR_STREAM( "Received incomplete reply stopping acquisition");
            return false;
        }

        frameControl = reply.data()[0];
        lenght = reply.data()[1];
        messId = reply.data()[2];

        if( frameControl == 0x80 && lenght==0x01 && messId == 0x53 )
        {
            ROS_INFO_STREAM( "Received ACK: IMU acquisition stopped");

            return true;
        }

        if( frameControl == 0xC0 && lenght==0x02 && messId == 0x53 )
        {
            ROS_ERROR_STREAM( "Received NACK: IMU acquisition not stopped. Error code: " << reply.data()[3] );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame" );
        return false;
    }

    return false;
}

}


