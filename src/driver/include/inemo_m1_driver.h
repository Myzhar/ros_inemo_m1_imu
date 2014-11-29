#ifndef _INEMO_M1_DRIVER_H_
#define _INEMO_M1_DRIVER_H_

#include <QThread>
#include <QMutex>
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

using namespace std;

#define BUFF_SIZE 128

namespace inemo
{

/*!
 * \struct _iNemoFrame
 *
 * \brief contains the communication frame (see doc UM1744 cap 1.1)
 */
typedef struct _iNemoFrame
{
    uint8_t mControl;   ///< Control byte
    uint8_t mLenght;     ///< Size byte
    uint8_t mId;        ///< Message id
    uint8_t mPayload[125];   ///< Frame payload
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
 * The class is derived by QThread (Qt5). A thread is started to
 * process inertial data emitted asynchronously by the IMU after
 * the command "iNEMO_Acquisition_Data". The thread is stopped/paused
 * to configure the board and to process synchronous command
 * and reply messages
 *
 */
class CInemoDriver : public QThread
{
    Q_OBJECT

public:

    typedef enum _dataFreq{ freq_1_hz  = 0x00,  // 000
                            freq_10_hz = 0x01,  // 001
                            freq_25_hz = 0x02,  // 010
                            freq_50_hz = 0x03,  // 011
                            freq_30_hz = 0x04,  // 100
                            freq_100_hz = 0x05, // 101
                            freq_400_hz = 0x06, // 110
                            sensor_sync = 0x07  // 111
                            } DataFreq;

    CInemoDriver();
    ~CInemoDriver();

    bool startIMU();
    bool stopIMU();

    std::string getFrameType( uint8_t ctrlByte);
    std::string getMsgName( uint8_t msgIdx );
    std::string getErrorString( uint8_t errIdx );
    std::string getFrequencyString( DataFreq freq );




protected:
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
    quint8 iNEMO_Get_MCU_ID();
    QString iNEMO_Get_FW_Version();
    QString iNEMO_Get_HW_Version();
    quint8 iNEMO_Identify();
    QString iNEMO_Get_AHRS_Library();
    bool iNEMO_Get_Libraries( bool& FAT, bool& Trace, bool& Altimeter, bool& Compass, bool& AHRS );
    bool iNEMO_Get_Available_Sensors( bool& acc, bool& gyro, bool& mag, bool& press, bool& temp );
    // <<<<< Board information frames

    // >>>>> Sensor setting frames
    //TODO try to understand the better way to implement this!
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
     * Process the serial data continously when
     * the \ref iNEMO_Start_Acquisition is called
     */
    void run() Q_DECL_OVERRIDE;

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
    ros::NodeHandle m_nh;

    serial::Serial mSerial;

    string mSerialPort;
    uint32_t mBaudrate;
    uint32_t mTimeout;

    bool mConnected; ///< indicates if \ref iNEMO_Connect() has been called

    bool mStopped; ///< Used to stop the serial processing
    bool mPaused; ///< Used to pause asynchronous data processing

    uint64_t mNoDataCounter;

    uint8_t mSerialBuf[BUFF_SIZE]; ///< Used to store bytes to be sent to serial port

    QMutex mMutex;

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
};

}

#endif

