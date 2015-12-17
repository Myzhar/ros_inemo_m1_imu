#include "inemo_m1_driver.h"

#include <bitset>
#include <byteswap.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

#define BIT(i) (1 << (i))
#define BIT_TEST(n, i) (((n) & BIT(i)) >> (i))

#define g 9.80665
#define RAD2DEG 57.295779513082320876798154814105
#define DEG2RAD 0.01745329251994329576923690768489

namespace inemo
{

bool CInemoDriver::mStopping = false;

CInemoDriver::CInemoDriver()
    : m_nhPriv("~")
{
    // >>>>> Ctrl+C handling
    struct sigaction sigAct;
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = CInemoDriver::sighandler;
    sigaction(SIGINT, &sigAct, 0);
    // <<<<< Ctrl+C handling

    mSerialPort = "/dev/ttyACM0";
    mBaudrate = 56000;
    mTimeout = 500;

    mConnected = false;
}

CInemoDriver::~CInemoDriver()
{

}

void CInemoDriver::loadParams()
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

    string fwVer = iNEMO_Get_FW_Version();

    ROS_INFO_STREAM( "Firmware version: " << fwVer );

    // >>>>> Data Configuration
    // TODO Load IMU configuration from params
    bool ahrs = true;
    bool compass = true;
    bool raw = false;       // Data always calibrated
    bool acc = true;
    bool gyro = true;
    bool mag = true;
    bool press = true;
    bool temp = true;
    bool continous = true; // Always continous streaming
    DataFreq freq = freq_50_hz;
    uint16_t samples = 0;

    iNEMO_Set_Output_Mode( ahrs, compass, raw, acc,
                           gyro, mag, press, temp,
                           continous, freq, samples );

    iNEMO_Get_Output_Mode( mAhrs, mCompass, mRaw, mAcc,
                           mGyro, mMag, mPress, mTemp,
                           mContinous, mFreq, mSamples );

    ROS_INFO_STREAM( "AHRS is " << mAhrs );
    ROS_INFO_STREAM( "COMPASS is " << mCompass );
    ROS_INFO_STREAM( "RAW is " << mRaw );
    ROS_INFO_STREAM( "ACCELEROMETER is " << mAcc );
    ROS_INFO_STREAM( "GYROSCOPE is " << mGyro );
    ROS_INFO_STREAM( "MAGNETOMETER is " << mMag );
    ROS_INFO_STREAM( "PRESSION is " << mPress );
    ROS_INFO_STREAM( "TEMPERATURE is " << mTemp );

    ROS_INFO_STREAM( "CONTINOUS MODE is " << mContinous );
    ROS_INFO_STREAM( "Data frequency is " << getFrequencyString(mFreq) );
    ROS_INFO_STREAM( "Samples count is " << mSamples );
    // <<<<< Data Configuration

    if( iNEMO_Start_Acquisition() )
    {
        iNEMO_Led_Control(true);

        //Thread start
        startThread();

        return true;
    }

    return false;
}

bool CInemoDriver::stopIMU()
{
    if( iNEMO_Stop_Acquisition() )
    {
        mStopped = true;

        iNEMO_Led_Control(false);
        iNEMO_Disconnect();
    }
}

void  CInemoDriver::startThread(void)
{
    int       result;
    result = pthread_create(&mThreadId, 0, CInemoDriver::callRunFunction, this);
    if (result == 0)
        pthread_detach(mThreadId);
}

std::string CInemoDriver::getFrequencyString( DataFreq freq )
{
    if( freq == freq_1_hz )
        return "1 hz";

    if( freq == freq_30_hz )
        return "30 hz";

    if( freq == freq_50_hz )
        return "50 hz";

    if( freq == freq_100_hz )
        return "100 hz";

    if( freq == freq_10_hz )
        return "10 hz";

    if( freq == freq_25_hz )
        return "25 hz";

    if( freq == freq_400_hz )
        return "400 hz";

    if( freq == sensor_sync )
        return "sensor_sync";
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
    //QMutexLocker locker(&mMutex);
    pthread_mutex_lock(&mMutex);

    bool reply;

    if(mPaused)
    {
        reply = iNEMO_Stop_Acquisition();
    }
    else
    {
        // call iNEMO_Start_Acquisition to restart the data reception
        reply = iNEMO_Start_Acquisition();
    }

    mPaused = paused;

    pthread_mutex_unlock(&mMutex);
    return mPaused;
}

float CInemoDriver::cast_and_swap_float(uint8_t* startAddr)
{
    float reply;
    uint32_t* cast = (uint32_t*)startAddr; // Casting uint8_t to uint32_t
    uint32_t swapped = bswap_32(*cast); // Swapping bytes and casting to float

    memcpy( &reply, &swapped, sizeof(float));

    return reply;
}

int16_t CInemoDriver::cast_and_swap_int16( uint8_t* startAddr )
{
    uint16_t* cast = (uint16_t*)startAddr; // Casting uint8_t to uint16_t
    int16_t swapped = (int16_t)bswap_16(*cast); // Swapping bytes and casting to int16_t

    return swapped;
}

int32_t CInemoDriver::cast_and_swap_int32( uint8_t* startAddr )
{
    uint32_t* cast = (uint32_t*)startAddr; // Casting uint8_t to uint32_t
    int32_t swapped = (int32_t)bswap_32(*cast); // Swapping bytes and casting to int32_t

    return swapped;
}

void* CInemoDriver::run()
{
    mStopped = false;
    mPaused = false;
    mNoDataCounter = 0;

    ROS_INFO_STREAM( "IMU Data acquiring loop started");

    static ros::Publisher imu_pub = m_nh.advertise<sensor_msgs::Imu>("imu_data", 10, false);
    static ros::Publisher mag_pub = m_nh.advertise<sensor_msgs::MagneticField>("inemo/mag", 10, false);
    static ros::Publisher rpy_pub = m_nh.advertise<geometry_msgs::Vector3Stamped>("inemo/rpy", 10, false);
    static ros::Publisher temp_pub = m_nh.advertise<sensor_msgs::Temperature>("inemo/temperature", 10, false);
    static ros::Publisher press_pub = m_nh.advertise<std_msgs::Float32>( "inemo/pressure", 10, false);

    std_msgs::Header header;
    ros::param::param<std::string>("~frame_id", header.frame_id, "imu_link");

    while(!mStopped)
    {
        ros::spinOnce(); // Processing ROS messages

        if( mStopping )
        {
            stopIMU();
            break;
        }

        if(!mPaused)
        {
            if( mSerial.waitReadable() )
            {
                ROS_DEBUG_STREAM( "---------------------------------" );
                ROS_DEBUG_STREAM( "Reading from serial port");

                int available = mSerial.available();
                string serialData = mSerial.read(available);
                ROS_DEBUG_STREAM( "Bytes available: " << available );


                iNemoFrame frame;
                if( processSerialData( serialData, &frame ) )
                {
                    if( frame.mControl == 0x40 && frame.mLenght>3 && frame.mId == 0x52 )
                    {
                        // >>>>> Control over size of data
                        int totByte = 3; // [Lenght][msg_id][counter 2 Byte]

                        if( mAcc )
                            totByte += 6;

                        if( mGyro )
                            totByte += 6;

                        if( mMag )
                            totByte += 6;

                        if( mPress )
                            totByte += 4;

                        if( mTemp )
                            totByte += 2;

                        if( mAhrs )
                            totByte += 28; // (12 bytes RPY + 16 bytes Quaternion)

                        if( mCompass )
                            totByte += 12; // 12 bytes, not 16 bytes as reported on the Communication Protocol document

                        if( frame.mLenght != totByte )
                        {
                            ROS_ERROR_STREAM( "The size of the frame is not correct. Received " << (unsigned short int)frame.mLenght << " bytes, expected: " << totByte );
                            continue;
                        }
                        // <<<<< Control over size of data

                        sensor_msgs::Imu imuMsg;
                        sensor_msgs::Temperature tempMsg;
                        sensor_msgs::MagneticField magFieldMsg;
                        geometry_msgs::Vector3Stamped rpyMsg;
                        std_msgs::Float32 pressMsg;

                        header.stamp = ros::Time::now();

                        int byteIndex = 0;

                        uint16_t dataCounter = bswap_16(*(uint16_t*)(&(frame.mPayload[byteIndex]))); // Casting uint8_t to uint16_t and byte swapping
                        ROS_DEBUG_STREAM( "Data counter: " << dataCounter );

                        byteIndex+=2;

                        if( mAcc )
                        {
                            float accX_g, accY_g, accZ_g;
                            double accX, accY, accZ;

                            int16_t valX = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            accX_g = (float)valX / 1000.0;
                            accX = accX_g*g;

                            byteIndex+=2; // Next value

                            int16_t valY = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            accY_g = (float)valY / 1000.0;
                            accY = accY_g*g;

                            byteIndex+=2; // Next value

                            int16_t valZ = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            accZ_g = (float)valZ / 1000.0;
                            accZ = accZ_g*g;

                            byteIndex+=2; // Next value

                            ROS_DEBUG_STREAM("Accelerations: " << accX << " m/sec^2, " << accY << " m/sec^2, " << accZ << " m/sec^2"  );

                            imuMsg.linear_acceleration.x = accX;
                            imuMsg.linear_acceleration.y = accY;
                            imuMsg.linear_acceleration.z = accZ;

                            // TODO add covariance matrix using data from sensor datasheet
                        }

                        if( mGyro )
                        {
                            float gyroX_deg, gyroY_deg, gyroZ_deg;
                            double gyroX, gyroY, gyroZ;

                            int16_t valX = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            gyroX_deg = (float)valX;
                            gyroX = gyroX_deg*DEG2RAD;

                            byteIndex+=2; // Next value

                            int16_t valY = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            gyroY_deg = (float)valY;
                            gyroY = gyroY_deg*DEG2RAD;

                            byteIndex+=2; // Next value

                            int16_t valZ = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            gyroZ_deg = (float)valZ;
                            gyroZ = gyroZ_deg*DEG2RAD;

                            byteIndex+=2; // Next value

                            ROS_DEBUG_STREAM("Rotations: " << gyroX << " rad/sec, " << gyroY << " rad/sec, " << gyroZ << " rad/sec" );

                            imuMsg.angular_velocity.x = gyroX;
                            imuMsg.angular_velocity.y = gyroY;
                            imuMsg.angular_velocity.z = gyroZ;

                            // TODO add covariance matrix using data from sensor datasheet
                        }

                        float magX, magY, magZ;

                        if( mMag )
                        {
                            // Magnetic Fields are expressed in milliGauss.
                            // To convert in Tesla we must multiply for 10
                            int16_t valX = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            //magX = (float)valX / 1000.0;
                            magX = (float)valX * 10.0;

                            byteIndex+=2; // Next value

                            int16_t valY = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            //magY = (float)valY / 1000.0;
                            magY = (float)valY * 10.0;

                            byteIndex+=2; // Next value

                            int16_t valZ = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            //magZ = (float)valZ / 1000.0;
                            magZ = (float)valZ * 10.0;

                            byteIndex+=2; // Next value

                            ROS_DEBUG_STREAM("Magnetic field: " << magX << " Tesla," << magY << " Tesla," << magZ << " Tesla" );

                            magFieldMsg.magnetic_field.x = magX;
                            magFieldMsg.magnetic_field.y = magY;
                            magFieldMsg.magnetic_field.z = magZ;

                            // TODO add covariance matrix using data from sensor datasheet
                        }

                        float pressure;
                        if( mPress )
                        {
                            int32_t val = cast_and_swap_int32( &(frame.mPayload[byteIndex]) );
                            pressure = (float)val / 100.0;

                            byteIndex+=4; // Next value

                            ROS_DEBUG_STREAM("Pressure: " << pressure << " mbar" );

                            pressMsg.data = pressure;
                        }

                        float temperature;
                        if( mTemp )
                        {
                            int16_t val = cast_and_swap_int16( &(frame.mPayload[byteIndex]) );
                            temperature = (float)val / 10.0;

                            byteIndex+=2; // Next value

                            ROS_DEBUG_STREAM("Temperature: " << temperature );

                            tempMsg.temperature = temperature;

                            // TODO add variance according to sensor datasheet
                        }

                        float R,P,Y;
                        float quatX, quatY, quatZ, quatW;

                        if( mAhrs )
                        {
                            // >>>>> Roll Pitch Yaw
                            R = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            P = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            Y = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            ROS_DEBUG_STREAM("Roll, Pitch, Yaw: " << R << " " << P << " " << Y );

                            rpyMsg.vector.x = R;
                            rpyMsg.vector.y = P;
                            rpyMsg.vector.z = Y;
                            // <<<<< Roll Pitch Yaw

                            // >>>>> quaternion
                            quatW = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            quatX = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            quatY = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            quatZ = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            ROS_DEBUG_STREAM("Q0, Q1, Q2, Q3: " << quatX << " " << quatY << " " << quatZ << " " << quatW );


                            imuMsg.orientation.x = quatX;
                            imuMsg.orientation.y = quatY;
                            imuMsg.orientation.z = quatZ;
                            imuMsg.orientation.w = quatW;
                            // <<<<< quaternion

                            // >>>>> Test
                            // This test is useful to verify that Quaternion and RPY are read correctly

                            //tf::Quaternion testQuat =  tf::createQuaternionFromRPY(R*DEG2RAD,P*DEG2RAD,Y*DEG2RAD);
                            //ROS_DEBUG_STREAM("Q0_T, Q1_T, Q2_T, Q3_T: " << testQuat.getX() << " " << testQuat.getY() << " " << testQuat.getZ() << " " << testQuat.getW() );
                            // <<<<< Test
                        }

                        float comR,comP,Head;

                        if( mCompass )
                        {
                            // >>>>> Compass Roll Pitch Heading
                            comR = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            comP = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            Head = cast_and_swap_float( &(frame.mPayload[byteIndex]) );
                            byteIndex+=4; // Next value

                            ROS_DEBUG_STREAM("Compass Roll, Pitch, Heading: " << comR << " " << comP << " " << Head );
                            // <<<<< Roll Pitch Heading
                        }

                        // >>>>> Message publishing
                        imuMsg.header = header;
                        rpyMsg.header = header;
                        magFieldMsg.header = header;
                        tempMsg.header = header;
                        //pressMsg.header = header;

                        if(imu_pub.getNumSubscribers() > 0)
                            imu_pub.publish( imuMsg );

                        if(rpy_pub.getNumSubscribers() > 0)
                            rpy_pub.publish( rpyMsg );

                        if( mag_pub.getNumSubscribers() > 0)
                            mag_pub.publish( magFieldMsg );

                        if( temp_pub.getNumSubscribers() > 0 )
                            temp_pub.publish( tempMsg );

                        if( press_pub.getNumSubscribers() > 0 )
                            press_pub.publish( pressMsg );
                        // <<<<< Message publishing
                    }
                }
            }
            else
            {
                mNoDataCounter++;
                ROS_INFO_STREAM( "No IMU data since more than " << mNoDataCounter * mTimeout << " msec" );
            }
        }
        else
            //msleep( 10 );
            usleep( 10000 );
    }

    ROS_INFO_STREAM( "IMU Data acquiring loop stopped");

    ROS_INFO_STREAM( "Shutting down the node...");
    ros::shutdown();

    return 0;
}

bool CInemoDriver::processSerialData(string& serialData , iNemoFrame *outFrame)
{
    int dataSize = serialData.size();

    // NOTE: According to Communication Protocol payload cannot me bigger than 61 bytes,
    //       but the version 2.5.0 does not transmit fragmented frames and payload
    //       are actually bigger than 61 bytes. Modify this function for future FW version
    //       that respects 61 bytes limitation.

    if( serialData.size()<3 || dataSize > 255 )
    {
        ROS_ERROR_STREAM( "The size of the frame is not correct. Received " << serialData.size() << " bytes (min3 bytes)" );
        return false;
    }

    outFrame->mControl = (uint8_t)serialData.data()[0];
    outFrame->mLenght = (uint8_t)serialData.data()[1];
    outFrame->mId = (uint8_t)serialData.data()[2];

    ROS_DEBUG_STREAM( "Frame received:" );
    ROS_DEBUG_STREAM( "Control:      0x" << std::hex  << std::setfill ('0') << std::setw(2) << (unsigned short int)outFrame->mControl << " - " << getFrameType( outFrame->mControl ) );
    ROS_DEBUG_STREAM( "Lenght:       0x" << std::hex  << std::setfill ('0') << std::setw(2) << (unsigned short int)outFrame->mLenght );
    ROS_DEBUG_STREAM( "Message Id:   0x" << std::hex  << std::setfill ('0') << std::setw(2) << (unsigned short int)outFrame->mId << " - " << getMsgName( outFrame->mId ) );
    ROS_DEBUG_STREAM( "Payload size: " << outFrame->mLenght-1 );

    if( outFrame->mLenght<1 || outFrame->mLenght>255 )
    {
        ROS_ERROR_STREAM( "Wrong frame lenght" );
        return false;
    }

    // NOTE: the following commented code must be enabled and modified if the FW supports fragmented
    //       frames. Actually version 2.5.0 does not transmit fragmented frames!

    /*bool bit_4 = BIT_TEST( outFrame->mControl , 4 );

    if( bit_4 ) // Multiframe
    {
        ROS_DEBUG_STREAM( "Fragmented frame! Total lenght: " << (int)(outFrame->mLenght) << " bytes" );

        int totFrame = std::ceil((double)(outFrame->mLenght)/61); // 61 is the max lenght of the single frame

        // Copying the first part of the payload
        memcpy( outFrame->mPayload, &serialData.data()[3], outFrame->mLenght-1 );

        int remaining = (outFrame->mLenght-1)-61; // tot bytes of payload to be downloaded from next fragments


        for( int i=1; i<totFrame; i++ ) // downloading remaining frames
        {
            if( !mSerial.waitReadable() )
            {
                ROS_ERROR_STREAM( "IMU timeout receiving fragmented frames");
                return false;
            }

            int bytesAvailable = mSerial.available();
            string reply = mSerial.read( bytesAvailable );

            uint8_t control = (uint8_t)reply.data()[0];
            if( control != outFrame->mControl )
            {
                ROS_ERROR_STREAM( "Fragment #" << i << " Control field (" << control
                                  << ") different from first fragment (" << outFrame->mControl << ")"  );
                return false;
            }

            uint8_t lenght = (uint8_t)reply.data()[1];
            if( lenght != outFrame->mLenght )
            {
                ROS_ERROR_STREAM( "Fragment #" << i << " Lenght field different from first fragment" );
                return false;
            }

            uint8_t id = (uint8_t)reply.data()[2];
            if( id != outFrame->mId )
            {
                ROS_ERROR_STREAM( "Fragment #" << i << " ID field different from first fragment" );
                return false;
            }

            bool bit_4 = BIT_TEST( control , 4 );

            if( bit_4 ) // not last fragment
            {
                // Copying the i-th part of the payload
                memcpy( &(outFrame->mPayload[61*i]), &reply.data()[3], 61 );

                remaining -= 61;
            }
            else // Last fragment
            {
                // Copying the last part of the payload
                memcpy( &(outFrame->mPayload[61*i]), &reply.data()[3], remaining );
            }
        }

    }
    else */
    if( outFrame->mLenght>1 ) // single frame
    {
        memcpy( outFrame->mPayload, &serialData.data()[3], outFrame->mLenght-3 );
    }

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
    //    for( int i=0; i<dataSize; i++ )
    //        ROS_DEBUG_STREAM( "[" << i << "]" << std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)mSerialBuf[i] );

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
        ROS_ERROR_STREAM( "Cannot connect, serial port non opened");
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
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

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
        ROS_ERROR_STREAM( "Cannot disconnect, serial port non opened");
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
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

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

bool CInemoDriver::iNEMO_Led_Control( bool enable )
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Led_Control' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_ERROR_STREAM( "Cannot connect, serial port non opened");
        return false;
    }

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x02;
    frame.mId = 0x08;
    frame.mPayload[0] = enable?1:0;

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout controlling LED");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return false;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x08 )
        {
            ROS_INFO_STREAM( "Received ACK: LED controlled");

            return true;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK: LED not connected. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        return false;
    }

    return false;
}

string CInemoDriver::iNEMO_Get_FW_Version()
{
    string fwStr("");

    ROS_INFO_STREAM( "Sending 'iNEMO_Get_FW_Version' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_ERROR_STREAM( "Cannot connect, serial port non opened");
        return fwStr;
    }

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x01;
    frame.mId = 0x13;

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout getting FW version");
            return fwStr;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return fwStr;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght>2 && replyFrame.mId == 0x13 )
        {
            ROS_INFO_STREAM( "Received ACK: FW correct");

            //fwStr.resize( replyFrame.mLenght );

            //memcpy( fwStr.data(), replyFrame.mPayload, replyFrame.mLenght );

            fwStr.assign( (char*)replyFrame.mPayload );

            return fwStr;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
            return fwStr;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        return fwStr;
    }

    return fwStr;
}

bool CInemoDriver::iNEMO_Set_Output_Mode(bool ahrs, bool compass, bool raw, bool acc,
                                         bool gyro, bool mag, bool press, bool temp,
                                         bool continousMode, DataFreq freq, uint16_t sampleCount )
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Set_Output_Mode' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_ERROR_STREAM( "Cannot send command, serial port non opened");
        return false;
    }

    if( !mStopped && !mPaused )
    {
        ROS_ERROR_STREAM( "IMU acquisition must be stopped or paused before changing configuration");
        return false;
    }

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x05;
    frame.mId = 0x50;
    //   Bit0      Bit1       Bit2       Bit3       Bit4       Bit5       Bit6       Bit7
    frame.mPayload[0] = 0x00; // | AHRS    | COMPASS  | Cal/Raw  | ACC      | GYRO     | MAG      | PRESS    | TEMP     |
    frame.mPayload[1] = 0x00; // | RFU0    | ASK_DATA | FQ02     | FQ1      | FQ0      | OT2      | OT1      | OT0      |
    frame.mPayload[2] = 0x00; // Number of samples MSB
    frame.mPayload[3] = 0x00; // Number of samples LSB

    if(ahrs)
        frame.mPayload[0] |= 0x80;
    if(compass)
        frame.mPayload[0] |= 0x40;
    if(raw)
        frame.mPayload[0] |= 0x20;
    if(acc)
        frame.mPayload[0] |= 0x10;
    if(gyro)
        frame.mPayload[0] |= 0x08;
    if(mag)
        frame.mPayload[0] |= 0x04;
    if(press)
        frame.mPayload[0] |= 0x02;
    if(temp)
        frame.mPayload[0] |= 0x01;

    if(!continousMode)
        frame.mPayload[1] |= 0x40;

    frame.mPayload[1] |= (uint8_t)(((uint8_t)freq) << 3);

    frame.mPayload[2] = (uint8_t)((sampleCount & 0xFF00) >> 8);
    frame.mPayload[3] = (uint8_t)(sampleCount & 0x00FF);

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout on iNEMO_Set_Output_Mode frame");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return false;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x50 )
        {
            ROS_INFO_STREAM( "Received ACK: IMU sensors configured");

            return true;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK: IMU sensors configured. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
            return false;
        }

        ROS_ERROR_STREAM( "Received unknown frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        return false;
    }

    return false;
}

bool CInemoDriver::iNEMO_Get_Output_Mode(bool& ahrs, bool& compass, bool& raw, bool& acc,
                                         bool& gyro, bool &mag, bool &press, bool& temp,
                                         bool& continousMode, DataFreq& freq, uint16_t& sampleCount )
{
    ROS_INFO_STREAM( "Sending 'iNEMO_Get_Output_Mode' frame to iNemo");

    if(!mSerial.isOpen())
    {
        ROS_ERROR_STREAM( "Cannot send command, serial port non opened");
        return false;
    }

    if( !mStopped && !mPaused )
    {
        ROS_ERROR_STREAM( "IMU acquisition must be stopped or paused before getting configuration");
        return false;
    }

    iNemoFrame frame;
    frame.mControl = 0x20;
    frame.mLenght = 0x01;
    frame.mId = 0x51;

    if( sendSerialCmd( frame ) )
    {
        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout on iNEMO_Get_Output_Mode frame");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

        iNemoFrame replyFrame;
        if( !processSerialData( reply, &replyFrame ) )
            return false;

        if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x05 && replyFrame.mId == 0x51 )
        {
            ROS_INFO_STREAM( "Received ACK: valid IMU sensors configuration");

            ahrs =      BIT_TEST(replyFrame.mPayload[0],7);
            compass =   BIT_TEST(replyFrame.mPayload[0],6);
            raw =       BIT_TEST(replyFrame.mPayload[0],5);
            acc =       BIT_TEST(replyFrame.mPayload[0],4);
            gyro =      BIT_TEST(replyFrame.mPayload[0],3);
            mag =       BIT_TEST(replyFrame.mPayload[0],2);
            press =     BIT_TEST(replyFrame.mPayload[0],1);
            temp =      BIT_TEST(replyFrame.mPayload[0],0);

            continousMode = (BIT_TEST(replyFrame.mPayload[1],6)==0)?true:false;

            freq = (DataFreq)((replyFrame.mPayload[1] >> 3) & 0x07);

            sampleCount = replyFrame.mPayload[2] * 256 + replyFrame.mPayload[3];

//            ROS_DEBUG_STREAM( "byte 0: " << (bitset<8>) replyFrame.mPayload[0] );
//            ROS_DEBUG_STREAM( "byte 1: " << (bitset<8>) replyFrame.mPayload[1] );
//            ROS_DEBUG_STREAM( "byte 2: " << (bitset<8>) replyFrame.mPayload[2] );
//            ROS_DEBUG_STREAM( "byte 3: " << (bitset<8>) replyFrame.mPayload[3] );

            return true;
        }

        if( replyFrame.mControl == 0xC0 )
        {
            uint8_t errorCode = replyFrame.mPayload[0];
            ROS_ERROR_STREAM( "Received NACK: IMU sensors error. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
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
        ROS_ERROR_STREAM( "Cannot start acquisition, serial port non opened");
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
        //        for( int i=0; i< reply.size(); i++ )
        //            ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (unsigned short int)reply.at(i) );

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
        ROS_ERROR_STREAM( "Cannot stop acquisition, serial port non opened");
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
            //            for( int i=0; i< reply.size(); i++ )
            //                ROS_DEBUG_STREAM( "[" << i << "]"<< std::hex << " 0x" << std::setfill ('0') << std::setw(2) << (short int)reply.at(i) );

            iNemoFrame replyFrame;
            processSerialData( reply, &replyFrame );

            if( replyFrame.mControl == 0x80 && replyFrame.mLenght==0x01 && replyFrame.mId == 0x53 )
            {
                ROS_INFO_STREAM( "Received ACK: IMU acquisition stopping confirmed");

                ROS_INFO_STREAM( "Receiving last data to flush buffer...");
            }
            else if( replyFrame.mControl == 0xC0 )
            {
                uint8_t errorCode = replyFrame.mPayload[0];
                ROS_ERROR_STREAM( "Received NACK: IMU acquisition not stopped. Error code: " << getMsgName(replyFrame.mId) << " - " << getErrorString( errorCode )  );
                return false;
            }
            else
                ROS_ERROR_STREAM( "Received wrong frame: " << std::hex << (int)reply.data()[0] << " " << std::hex << (int)reply.data()[1] << " "<< (int)reply.data()[2] << " "<< (int)reply.data()[3] << " "<< (int)reply.data()[4] );
        }

        ROS_INFO_STREAM( "Data acquisition stopped");

        return true;
    }

    return false;
}

}


