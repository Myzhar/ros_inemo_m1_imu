#include <inemo_m1_driver.h>

#include <QMutexLocker>

#define BIT(i) (1 << (i))
#define BIT_TEST(n, i) (((n) & BIT(i)) >> (i))

namespace inemo
{

CInemoDriver::CInemoDriver() :
    QThread( NULL )
{
    // TODO: add parameters for serial port initialization!!!

    mSerialPort = "/dev/ttyACM0";
    mBaudrate = 56000;
    mTimeout = 500;

    mConnected = false;
}

CInemoDriver::~CInemoDriver()
{

}

bool CInemoDriver::startIMU()
{
    // TODO: replace with serial parameters

    ROS_INFO_STREAM( "Opening serial port " << mSerialPort );

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

    ROS_INFO_STREAM( "Serial port ready" );

    // TODO Configurate IMU before starting acquisition

    // >>>>> Disconnection to resolve previous bad stopping

    ROS_INFO_STREAM( "Trying to stop Data Acquisition to solve previously bad interruption of the node"  );
    stopIMU();
    // <<<<< Disconnection to resolve previous bad stopping

    if( !iNEMO_Connect() )
    {
        return false;
    }

    mPaused = false;
    mStopped = true;

    if( iNEMO_Start_Acquisition() )
    {
        //Thread start
        start();

        return true;
    }

    return false;
}

bool CInemoDriver::stopIMU()
{
    if( iNEMO_Stop_Acquisition() )
    {
        mStopped = true;

        iNEMO_Disconnect();
    }
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

std::string CInemoDriver::getFrameType( uint8_t ctrlByte)
{
    bool bit_7 = BIT_TEST( ctrlByte, 7 );
    bool bit_6 = BIT_TEST( ctrlByte, 6 );

    if( !bit_7 && !bit_6 )
        return "Control";

    if( !bit_7 && bit_6 )
        return "Data";

    if( bit_7 && !bit_6 )
        return "Ack";

    if( bit_7 && bit_6 )
        return "Nack";
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
            if( mSerial.waitReadable() )
            {
                ROS_INFO_STREAM("Reading from serial port");

                string serialData = mSerial.read(mSerial.available());

                iNemoFrame frame;
                if( processSerialData( serialData, &frame ) )
                {
                    ROS_INFO_STREAM( "Data counter: " << (int)frame.mPayload[1] + (int )frame.mPayload[0]*256 );
                }
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

bool CInemoDriver::processSerialData(string& serialData , iNemoFrame *outFrame)
{
    if( serialData.size() <3 && serialData.size()>64 )
    {
        ROS_ERROR_STREAM( "The size of the frame is not correct. Received " << serialData.size() << " bytes" );
        return false;
    }

    outFrame->mControl = (uint8_t)serialData.data()[0];
    outFrame->mLenght = (uint8_t)serialData.data()[1];
    outFrame->mId = (uint8_t)serialData.data()[2];

    if(outFrame->mLenght > 126)
    {
        ROS_ERROR_STREAM( "The 'lenght' byte is not correct. Value:" << (int)outFrame->mLenght );
        return false;
    }

    if( outFrame->mLenght>1 )
    {
        memcpy( outFrame->mPayload, &serialData.data()[3], outFrame->mLenght-1 );
    }

    ROS_INFO_STREAM( "Frame received:" );
    ROS_INFO_STREAM( "Control:      0x" << std::hex  << std::setfill ('0') << std::setw(2) << (unsigned short int)outFrame->mControl << " - " << getFrameType( outFrame->mControl ) );
    ROS_INFO_STREAM( "Lenght:       0x" << std::hex  << std::setfill ('0') << std::setw(2) << (unsigned short int)outFrame->mLenght );
    ROS_INFO_STREAM( "Message Id:   0x" << std::hex  << std::setfill ('0') << std::setw(2) << (unsigned short int)outFrame->mId << " - " << getMsgName( outFrame->mId ) );
    ROS_INFO_STREAM( "Payload size: " << outFrame->mLenght-1 );

    return true;
}

bool CInemoDriver::sendSerialCmd( iNemoFrame &frame )
{
    mSerialBuf[0] = frame.mControl;
    mSerialBuf[1] = frame.mLenght;
    mSerialBuf[2] = frame.mId;

    int dataSize = frame.mLenght+2;

    if(frame.mLenght > 1)
        memcpy( &mSerialBuf[3], frame.mPayload, frame.mLenght-1 );

    ROS_DEBUG_STREAM( "To be written " << dataSize << " bytes" );
    for( int i=0; i<dataSize; i++ )
        ROS_DEBUG_STREAM( "[" << i << "]" << std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)mSerialBuf[i] );

    int written = mSerial.write( mSerialBuf, dataSize );
    if ( written != dataSize )
    {
        ROS_ERROR_STREAM( "Serial write error. Written " << written << " bytes instead of" << dataSize << " bytes.");
        return false;
    }

    return true;
}

bool CInemoDriver::iNEMO_Connect()
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Connect' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_INFO_STREAM( "Cannot connect, serial port non opened");
        return false;
    }

    if( mConnected )
    {
        ROS_ERROR_STREAM( "Board is connected. Command ignored");
        return false;
    }

    mConnected = false;

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x01;
    frame.mId = 0x00;

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout connecting");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        for( int i=0; i< reply.size(); i++ )
            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return false;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x00 )
        {
            ROS_INFO_STREAM( "Received ACK: IMU connected");

            mConnected = true;
            return true;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK: IMU not connected. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        return false;
    }

    return false;
}

bool CInemoDriver::iNEMO_Disconnect()
{
    ROS_INFO_STREAM( "Stopping eventually running acquisition...");

    iNEMO_Stop_Acquisition();

    ROS_INFO_STREAM( "Sending 'iNEMO_Disconnect' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_INFO_STREAM( "Cannot disconnect, serial port non opened");
        return false;
    }

    mConnected = false;

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x01;
    frame.mId = 0x01;

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout disconnecting");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        for( int i=0; i< reply.size(); i++ )
            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return false;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x00 )
        {
            ROS_INFO_STREAM( "Received ACK: IMU disconnected");

            mConnected = true;
            return true;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK: IMU not disconnected. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        return false;
    }

    return false;
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

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x01;
    frame.mId = 0x52;

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout starting acquisition");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        for( int i=0; i< reply.size(); i++ )
            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return false;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x52 )
        {
            ROS_INFO_STREAM( "Received ACK: IMU acquisition started");

            return true;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK: IMU acquisition not started. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        return false;
    }

    return false;
}

bool CInemoDriver::iNEMO_Stop_Acquisition()
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Stop_Acquisition' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_INFO_STREAM( "Cannot stop acquisition, serial port non opened");
        return false;
    }

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x01;
    frame.mId = 0x53;

    if( sendSerialCmd( frame ) )
    {
        while( mSerial.waitReadable() )
        {
            string reply = mSerial.read( mSerial.available() );

            ROS_DEBUG_STREAM( "Received " << reply.size() << "bytes" );
            for( int i=0; i< reply.size(); i++ )
                ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (short int)reply.at(i) );

            iNemoFrame replyFrame;
            processSerialData( reply, &replyFrame );

            if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x53 )
            {
                ROS_INFO_STREAM( "Received ACK: IMU acquisition stopping confirmed");

                ROS_INFO_STREAM( "Receiving last data to flush buffer...");
            }

            if( replyFrame.mControl == 0xC0 )
            {
                uint8_t errorCode = replyFrame.mPayload[0];
                ROS_ERROR_STREAM( "Received NACK: IMU acquisition not stopped. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
                return false;
            }

            ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        }

        ROS_INFO_STREAM( "Data acquisition stopped");
    }

    return false;
}

}


