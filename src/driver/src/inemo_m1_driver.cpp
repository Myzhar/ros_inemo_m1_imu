#include <inemo_m1_driver.h>

#include <QMutexLocker>

namespace inemo
{

CInemoDriver::CInemoDriver() :
    QThread( NULL )
{
    // TODO: add parameters for serial port initialization!!!

    mSerialPort = "/dev/ttyACM0";
    mBaudrate = 56000;
    mTimeout = 500;
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
        mSerial.open();
        //mSerial.setBaudrate(mBaudrate);

        serial::Timeout to = serial::Timeout::simpleTimeout(mTimeout);
        mSerial.setTimeout(to);
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

    mPaused = false;
    mStopped = true;

    return iNEMO_Start_Acquisition();
}

bool CInemoDriver::stopIMU()
{
    mStopped = true;
}

std::string CInemoDriver::getErrorString( uint8_t errIdx )
{
    switch( errIdx )
    {
    case 0x00:
        return "Forbidden";

    case 0x01:
        return "Unsupported command";

    case 0x02:
        return "Out-of-range value";

    case 0x03:
        return "Not executable command";

    case 0x04:
        return "Wrong syntax";

    case 0x05:
        return "Discovery-M1 not connected";

    default:
        return "Unknown error";
    }
}

std::string CInemoDriver::getMsgName( uint8_t msgIdx )
{
    switch(msgIdx)
    {
    case 0x00:
        return "Connect";

    case 0x01:
        return "Disconnect";

    case 0x02:
        return "Reset_Board";

    case 0x03:
        return "Enter_DFU_Mode";

    case 0x07:
        return "Trace";

    case 0x08:
        return "Led_Control";

    case 0x10:
        return "Get_Device_Mode";

    case 0x12:
        return "Get_MCU_ID";

    case 0x13:
        return "Get_FW_Version";

    case 0x14:
        return "Get_HW_Version";

    case 0x15:
        return "Identify";

    case 0x17:
        return "Get_AHRS_Library";

    case 0x18:
        return "Get_Libraries";

    case 0x19:
        return "Get_Available_Sensors";

    case 0x20:
        return "Set_Sensor_Parameter";

    case 0x21:
        return "Get_Sensor_Parameter";

    case 0x22:
        return "Restore_Default_Parameter";

    case 0x23:
        return "Save_to_Flash";

    case 0x24:
        return "Load_from_Flash";

    case 0x50:
        return "Set_Output_Mode";

    case 0x51:
        return "Get_Output_Mode";

    case 0x52:
        return "Acquisition";

    case 0x53:
        return "Stop_Acquisition";

    case 0x54:
        return "Get_Acq_Data";

    default:
        return "Message ID unknown!";
    }
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

    ROS_INFO_STREAM( "Written " << dataSize+3 << "bytes" );
    for( int i=0; i< dataSize+3; i++ )
        ROS_INFO_STREAM( "[" << i << "]" << std::hex << " 0x" << (short int)mSerialBuf[i] );

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

        ROS_DEBUG_STREAM( "\rReceived " << reply.size() << "bytes" );
        for( int i=0; i< reply.size(); i++ )
            ROS_INFO_STREAM( "[" << i << "]" << std::setw(2) << std::hex << std::showbase << (short int)reply.at(i) );

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
            ROS_ERROR_STREAM( "Received NACK: IMU acquisition not started. Error code: " << std::hex << reply.data()[3] );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
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


