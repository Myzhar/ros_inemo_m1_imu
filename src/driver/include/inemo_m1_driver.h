#ifndef _INEMO_M1_DRIVER_H_
#define _INEMO_M1_DRIVER_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <signal.h>
#include <vector>

#include "cmobilewinmean.h"

using namespace std;

#define BUFF_SIZE 64

namespace inemo
{

/*!
 * \struct _iNemoFrame
 *
 * \brief contains the communication frame (see doc UM1744 cap 1.1)
 */
typedef struct _iNemoFrame
{
    uint8_t mControl; ///< Control byte
    uint8_t mLenght;  ///< Size byte
    uint8_t mId;      ///< Message id
    uint8_t mPayload[256];   ///< Frame payload, allows multi-frame fragmented data
} iNemoFrame;

/*!
 * \class CInemoDriver
 *
 * \brief The CInemoDriver class implements the ROS node to
 * communicate with the iNemo-M1 Discovery board (STEVAL-MKI121V1)
 * and to retrieve inertial information from the original firmware
 * preinstalled on the board.
 *
 * The class implements the protocol described in the document
 * "UM1744: STEVAL- MKI121V1 communication protocol" by ST.
 * A thread is started to process inertial data emitted asynchronously
 * by the IMU after the command "iNEMO_Acquisition_Data".
 * The thread is stopped/paused to reconfigure the board and to
 * process synchronous command and reply messages.
 *
 */
class CInemoDriver
{
public:

    typedef enum _dataFreq
    {
        freq_1_hz  = 0x00,  // 000
        freq_10_hz = 0x01,  // 001
        freq_25_hz = 0x02,  // 010
        freq_50_hz = 0x03,  // 011
        freq_30_hz = 0x04,  // 100
        freq_100_hz = 0x05, // 101
        freq_400_hz = 0x06, // 110
        sensor_sync = 0x07  // 111
    } DataFreq;

    typedef enum _sensorType
    {
        accel = 0x00,
        mag = 0x01,
        gyro = 0x02,
        press = 0x04,
        temp = 0x05
    } SensType;

    typedef enum _gyroParams
    {
        gyro_data_rate = 0x00,
        gyro_full_scale = 0x01,
        gyro_HPF = 0x02,
        gyro_offset_X = 0x03,
        gyro_offset_Y = 0x04,
        gyro_offset_Z = 0x05,
        gyro_scale_factor_X = 0x06,
        gyro_scale_factor_Y = 0x07,
        gyro_scale_factor_Z = 0x08,
        gyro_sensor_name = 0xFF
    } GyroParams;

    CInemoDriver();
    ~CInemoDriver();
    
    /// Function used to start a thread inside the class
    static void*  callRunFunction(void *arg) { return ((CInemoDriver*)arg)->run(); }
    
    void startThread(void);

    bool startIMU();
    bool stopIMU();

    std::string getFrameType( uint8_t ctrlByte);
    std::string getMsgName( uint8_t msgIdx );
    std::string getErrorString( uint8_t errIdx );
    std::string getFrequencyString( DataFreq freq );

protected:
    // >>>>> Ctrl+C handler
    /*! Ctrl+C handler
     */
    static void sighandler(int signo)
    {
        CInemoDriver::mStopping = (signo == SIGINT);
        ROS_INFO_STREAM("Ctrl+C pressed by user" );
    }
    // <<<<< Ctrl+C handler

    // >>>>> iNemo Protocol

    // >>>>> Communication control frames
    bool iNEMO_Connect();
    bool iNEMO_Disconnect();
    bool iNEMO_Reset();
    // bool iNEMO_Enter_DFU_Mode(); // Not implemented
    // bool iNEMO_Trace(); // Not implemented
    bool iNEMO_Led_Control( bool enable );
    // <<<<< Communication control frames

    // >>>>> Board information frames
    uint8_t iNEMO_Get_MCU_ID();
    string iNEMO_Get_FW_Version();
    string iNEMO_Get_HW_Version();
    uint8_t iNEMO_Identify();
    string iNEMO_Get_AHRS_Library();
    bool iNEMO_Get_Libraries( bool& FAT, bool& Trace, bool& Altimeter, bool& Compass, bool& AHRS );
    bool iNEMO_Get_Available_Sensors( bool& acc, bool& gyro, bool& mag, bool& press, bool& temp );
    // <<<<< Board information frames

    // >>>>> Sensor setting frames
    bool iNEMO_Set_Gyro_Offsets( int16_t offsetX, int16_t offsetY, int16_t offsetZ );

    // <<<<< Sensor setting frames

    // >>>>> Acquisition sensor data frames
    bool iNEMO_Set_Output_Mode(bool ahrs, bool compass, bool raw, bool acc,
                               bool gyro, bool mag, bool press, bool temp,
                               bool continousMode, DataFreq freq, uint16_t sampleCount );
    bool iNEMO_Get_Output_Mode( bool& ahrs, bool& compass, bool& raw, bool& acc,
                                bool& gyro, bool &mag, bool &press, bool& temp,
                                bool& continousMode, DataFreq& freq, uint16_t& sampleCount );
    bool iNEMO_Start_Acquisition(); ///< Starts asynchronous data acquisition according to \ref iNEMO_Set_Output_Mode params
    bool iNEMO_Stop_Acquisition();  ///< Stops asynchronous data acquisition
    bool iNEMO_Get_Acquired_Data(); ///< Request IMU data if asynchronous mode is note active
    // <<<<< Acquisition sensor data frames

    // <<<<< iNemo Protocol

    /*!
     * \brief pauseIMU, pauses the asynchronous data
     * processing to send synchronous commands.
     * The main thread is not stopped to continue
     * to receive ROS commands.
     *
     * \param paused, indicates if the pause is active or not
     *
     * \return returns pause status
     */
    bool pauseIMU( bool paused );

    /*!
     * \brief run
     * Thread function. Process the serial data continously when
     * the \ref iNEMO_Start_Acquisition is called
     */
    void* run();

    /*!
     * \brief processSerialData the data read from serial
     * port and returns the protocol \ref iNemoFrame
     *
     * \param serialData to be processed
     * \param outFrame the frame extracted from serial buffer
     *
     * \return true if the output frame is correct
     */
    bool processSerialData(string& serialData, iNemoFrame* outFrame );


    /*!
     * \brief sendSerialCmd sends a serial frame to iNemo Board
     *
     * \param frameControl Frame control according to protocol
     * \param lenght size of \ref payload + \ref messId in bytes
     * \param messId ID of the messages
     * \param payload optional payload of the frame
     *
     * \return true if success
     */
    bool sendSerialCmd( iNemoFrame& frame );

private:
    // >>>>> Conversion functions
    float cast_and_swap_float( uint8_t* startAddr );
    int16_t cast_and_swap_int16( uint8_t* startAddr );
    //void cast_and_swap_int16( int16_t val, uint8_t* outAddr );
    int32_t cast_and_swap_int32( uint8_t* startAddr );
    // <<<<< Conversion functions

    /*!
     * \brief loadParams load params from ROS param server
     */
    void loadParams();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nhPriv;

    serial::Serial mSerial;

    string mSerialPort;
    uint32_t mBaudrate;
    uint32_t mTimeout;

    bool mConnected; ///< indicates if \ref iNEMO_Connect() has been called

    // >>>>> IMU calibration
    bool mCalibActive; ///< indicates that the IMU is calibrating
    vector<float> mGyroX_vec;
    vector<float> mGyroY_vec;
    vector<float> mGyroZ_vec;
    float mGyroX_sum;
    float mGyroY_sum;
    float mGyroZ_sum;
    // <<<<< IMU calibration

    bool mStopped; ///< Used to stop the serial processing
    bool mPaused; ///< Used to pause asynchronous data processing

    static bool mStopping; ///< Used to stop driver using Ctrl+C

    uint64_t mNoDataCounter; ///< Consecutive thread loops without data from IMU

    uint8_t mSerialBuf[BUFF_SIZE]; ///< Used to store bytes to be sent to serial port

    // >>>>> Configuration
    bool mAhrs;
    bool mCompass;
    bool mRaw;
    bool mAcc;
    bool mGyro;
    bool mMag;
    bool mPress;
    bool mTemp;
    bool mContinous;
    DataFreq mFreq;
    uint16_t mSamples;
    // <<<<< Configuration

    // >>>>> Outliers rejection
    CMobileWinMean mFilterGyroX;
    CMobileWinMean mFilterGyroY;
    CMobileWinMean mFilterGyroZ;
    CMobileWinMean mFilterAccX;
    CMobileWinMean mFilterAccY;
    CMobileWinMean mFilterAccZ;
    // <<<<< Outliers rejection
    
    pthread_t mThreadId; // Thread Id
    //pthread_mutex_t mMutex; // Mutex
};

}

#endif

