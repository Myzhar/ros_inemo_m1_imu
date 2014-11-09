#ifndef _INEMO_M1_DRIVER_H_
#define _INEMO_M1_DRIVER_H_

#include <QThread>
#include <QMutex>
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

using namespace std;

namespace inemo
{

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
    CInemoDriver();
    ~CInemoDriver();

    bool startIMU();
    bool stopIMU();

    typedef enum _dataFreq{ freq_1_hz  = 0x00,
                            freq_10_hz = 0x01,
                            freq_25_hz = 0x02,
                            freq_50_hz = 0x03,
                            freq_30_hz = 0x04,
                            freq_100_hz = 0x05,
                            freq_400_hz = 0x06,
                            sensor_sync = 0x07
                            } DataFreq;

protected:
    // >>>>> iNemo Protocol

    // >>>>> Communication control frames
    bool iNEMO_Connect();
    void iNEMO_Disconnect();
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
    bool iNEMO_Set_Output_Mode( bool ahrs, bool compass, bool calib, bool acc,
                                bool gyro, bool mag, bool press, bool temp,
                                bool continousMode, DataFreq freq, int sampleCount );
    bool iNEMO_Get_Output_Mode( bool& ahrs, bool& compass, bool& calib, bool& acc,
                                bool& gyro, bool &mag, bool &press, bool& temp,
                                bool& continousMode, DataFreq& freq, int& sampleCount );
    bool iNEMO_Start_Acquisition(); ///< Starts asynchronous data acquisition according to \ref iNEMO_Set_Output_Mode params
    bool iNEMO_Stop_Acquisition(); ///< Stops asynchronous data acquisition
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
     */
    void pauseIMU( bool paused );

    /*!
     * \brief run
     * Process the serial data continously when
     * the \ref iNEMO_Start_Acquisition is called
     */
    void run() Q_DECL_OVERRIDE;

    /*!
     * \brief processSerialData processes the IMU
     * data received asynchrounsly.
     *
     * \param data to be processed
     */

    void processSerialData( string& data );

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
    bool sendSerialCmd( quint8 frameControl, quint8 lenght, quint8 messId, QByteArray payload );

private:
    ros::NodeHandle m_nh;

    serial::Serial mSerial;

    string mSerialPort;
    uint32_t mBaudrate;
    uint32_t mTimeout;

    bool mStopped; ///< Used to stop the serial processing
    bool mPaused; ///< Used to pause asynchronous data processing

    uint64_t mNoDataCounter;

    QMutex mMutex;
};

}

#endif

